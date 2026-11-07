
/**
 * @file main.c
 * @brief Interrupt-driven line follower and maze navigation for ECE 528/L Lab 3.
 *
 * The MSP432 samples the QTRX array every 10 ms, follows the black line,
 * explores both branches of the T-intersection, and returns to the starting
 * dead end. This controller targets the example maze in the Lab 3 handout.
 * Turning times and sensor thresholds require physical calibration.
 *
 * @author Carlos Bautista
 */

#include <stdint.h>
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

// These values are PWM timer counts, not percentages.
#define PWM_NOMINAL              2500
#define PWM_CORRECTION           1000
#define POSITION_GAIN_DIVISOR      48

// Starting calibration values (milliseconds).
#define TURN_90_MIN_MS           180
#define TURN_180_MIN_MS          550
#define TURN_TIMEOUT_MS         1800
#define CORNER_IGNORE_MS         130
#define CROSS_INTERSECTION_MS    180
#define MIN_BRANCH_MS            350
#define MIN_RETURN_MS           1500
#define DEAD_END_SAMPLES           3
#define TURN_CENTER_SAMPLES        2

static uint32_t Timer_A1_ms_elapsed = 0;

typedef enum
{
    CENTER              = 0b00011000,
    SLIGHT_LEFT_CENTER  = 0b00011100,
    SLIGHT_RIGHT_CENTER = 0b00111000,
    FAR_LEFT            = 0b00000001,
    FAR_RIGHT           = 0b10000000,
    LEFT_TURN           = 0b11111000,
    RIGHT_TURN          = 0b00011111,
    T_INTERSECTION      = 0b11111111,
    DEAD_END            = 0b00000000
} Line_Position;

typedef enum
{
    EXPLORE_MAZE,
    EXPLORE_INTERSECTION,
    RETURN_TO_START,
    FINISHED
} Navigation_State;

typedef enum
{
    LEFT_BRANCH_OUT,
    LEFT_BRANCH_BACK,
    RIGHT_BRANCH_OUT,
    RIGHT_BRANCH_BACK
} Intersection_Phase;

typedef enum
{
    NO_TURN,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    TURN_LEFT_180
} Turn_State;

static Navigation_State navigation_state = EXPLORE_MAZE;
static Intersection_Phase intersection_phase = LEFT_BRANCH_OUT;
static Turn_State turn_state = NO_TURN;

static uint32_t phase_start_ms = 0;
static uint32_t turn_start_ms = 0;
static uint32_t last_turn_finish_ms = 0;
static uint8_t dead_end_count = 0;
static uint8_t turn_center_count = 0;
static uint8_t branch_line_seen = 0;
static uint8_t navigation_error = 0;

/**
 * @brief Displays the exact lab-guide sensor patterns on the onboard LEDs.
 */
void Detect_Line_Position(uint8_t data)
{
    LED1_Output(0x00);
    LED2_Output(0x00);

    switch (data)
    {
        case CENTER:
            LED2_Output(0x02);
            break;

        case SLIGHT_LEFT_CENTER:
            LED2_Output(0x03);
            break;

        case SLIGHT_RIGHT_CENTER:
            LED2_Output(0x05);
            break;

        case FAR_LEFT:
            LED2_Output(0x07);
            break;

        case FAR_RIGHT:
            LED2_Output(0x06);
            break;

        case LEFT_TURN:
            LED1_Output(0x01);
            LED2_Output(0x07);
            break;

        case RIGHT_TURN:
            LED1_Output(0x01);
            LED2_Output(0x06);
            break;

        case T_INTERSECTION:
            LED2_Output(0x04);
            break;

        case DEAD_END:
            LED2_Output(0x01);
            break;

        default:
            break;
    }
}

/**
 return Number of sensors detecting black, from 0 to 8.
 */
static uint8_t Count_Black_Sensors(uint8_t data)
{
    uint8_t count = 0;
    uint8_t bit;

    for (bit = 0; bit < 8; bit++)
    {
        if (data & (0x01u << bit))
        {
            count++;
        }
    }

    return count;
}

/**
 * Checks for a T-intersection using the reflectance sensor pattern.
 *
 * Detects an all-black reading or a sufficiently wide black region that
 * includes both outer sensors and at least one center sensor.
 */
static uint8_t Is_T_Intersection(uint8_t data)
{
    return (data == T_INTERSECTION) ||
           (((data & 0x81u) == 0x81u) &&
            ((data & 0x18u) != 0) &&
            (Count_Black_Sensors(data) >= 6));
}

/**
 * @brief Checks whether the sensor pattern indicates a left corner.
 */
static uint8_t Is_Left_Corner(uint8_t data)
{
    return ((data & 0xE0u) == 0xE0u) &&
           ((data & 0x18u) != 0) &&
           ((data & 0x03u) == 0);
}

/**
 * Checks whether the sensor pattern indicates a right corner.
 */
static uint8_t Is_Right_Corner(uint8_t data)
{
    return ((data & 0x07u) == 0x07u) &&
           ((data & 0x18u) != 0) &&
           ((data & 0xC0u) == 0);
}

/**
 * Stops if no black line is detected. Otherwise, calculates a limited
 * correction from the line position and rotates toward the line when
 * the deviation is large.
 */
void Line_Follower_Controller(uint8_t data)
{
    int32_t position;
    int32_t correction;

    if (data == DEAD_END)
    {
        Motor_Stop();
        return;
    }

    position = Reflectance_Sensor_Position(data);

    correction = (position * PWM_CORRECTION) /
                 POSITION_GAIN_DIVISOR;

    if (correction > PWM_CORRECTION)
    {
        correction = PWM_CORRECTION;
    }
    else if (correction < -PWM_CORRECTION)
    {
        correction = -PWM_CORRECTION;
    }

    if (position >= 250)
    {
        Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
    }
    else if (position <= -250)
    {
        Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
    }
    else
    {
        Motor_Forward(
            (uint16_t)(PWM_NOMINAL + correction),
            (uint16_t)(PWM_NOMINAL - correction)
        );
    }
}

/**
 * Records the turn start time, clears detection counters, and commands
 * the motors to rotate in the requested direction.
 */
static void Start_Turn(Turn_State requested_turn)
{
    turn_state = requested_turn;
    turn_start_ms = Timer_A1_ms_elapsed;
    turn_center_count = 0;
    dead_end_count = 0;

    if (turn_state == TURN_RIGHT_90)
    {
        Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
    }
    else
    {
        Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
    }
}

/**
 * Uses a minimum turn duration and two consecutive centered readings
 * to finish a turn. If the turn times out, the robot stops and enters
 * the FINISHED state with an error flag.
 */
static void Execute_Turn(uint8_t data)
{
    uint32_t elapsed =
        Timer_A1_ms_elapsed - turn_start_ms;

    uint32_t minimum =
        (turn_state == TURN_LEFT_180) ?
        TURN_180_MIN_MS : TURN_90_MIN_MS;

    if (elapsed >= TURN_TIMEOUT_MS)
    {
        navigation_error = 1;
        navigation_state = FINISHED;
        turn_state = NO_TURN;

        Motor_Stop();
        return;
    }

    if ((elapsed >= minimum) &&
        ((data & 0x18u) == 0x18u) &&
        !Is_T_Intersection(data) &&
        !Is_Left_Corner(data) &&
        !Is_Right_Corner(data))
    {
        if (turn_center_count < TURN_CENTER_SAMPLES)
        {
            turn_center_count++;
        }
    }
    else
    {
        turn_center_count = 0;
    }

    if (turn_center_count >= TURN_CENTER_SAMPLES)
    {
        turn_state = NO_TURN;
        last_turn_finish_ms = Timer_A1_ms_elapsed;
        phase_start_ms = Timer_A1_ms_elapsed;
        dead_end_count = 0;
        branch_line_seen = 1;

        Line_Follower_Controller(data);
        return;
    }

    if (turn_state == TURN_RIGHT_90)
    {
        Motor_Right(PWM_NOMINAL, PWM_NOMINAL);
    }
    else
    {
        Motor_Left(PWM_NOMINAL, PWM_NOMINAL);
    }
}

/**
 * Confirms a dead end using consecutive all-white readings.
 *
 */
static uint8_t Dead_End_Confirmed(uint8_t data)
{
    if (data == DEAD_END)
    {
        if (dead_end_count < DEAD_END_SAMPLES)
        {
            dead_end_count++;
        }
    }
    else
    {
        dead_end_count = 0;
    }

    return (dead_end_count >= DEAD_END_SAMPLES);
}

/**
 * Ignores repeated corner detections briefly after a turn, follows the
 * line using motor PWM corrections, and drives forward across a wide
 * intersection reading.
 *
 */
static void Follow_Track(uint8_t data)
{
    if ((Timer_A1_ms_elapsed - last_turn_finish_ms)
        >= CORNER_IGNORE_MS)
    {
        if (Is_Left_Corner(data))
        {
            Start_Turn(TURN_LEFT_90);
            return;
        }

        if (Is_Right_Corner(data))
        {
            Start_Turn(TURN_RIGHT_90);
            return;
        }
    }

    if (Is_T_Intersection(data))
    {
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
    }
    else
    {
        Line_Follower_Controller(data);
    }
}

/**
 * Follows the initial path, explores the left and right T-intersection
 * branches, returns along the original route, and stops at the starting
 * dead end. Active turns are completed before normal state processing.
 *
 */
void Navigation_Controller(uint8_t data)
{
    uint32_t elapsed =
        Timer_A1_ms_elapsed - phase_start_ms;

    if (navigation_state == FINISHED)
    {
        Motor_Stop();
        return;
    }

    if (turn_state != NO_TURN)
    {
        Execute_Turn(data);
        return;
    }

    switch (navigation_state)
    {
        case EXPLORE_MAZE:

            if (Is_T_Intersection(data))
            {
                navigation_state = EXPLORE_INTERSECTION;
                intersection_phase = LEFT_BRANCH_OUT;

                Start_Turn(TURN_LEFT_90);
            }
            else
            {
                Follow_Track(data);
            }
            break;

        case EXPLORE_INTERSECTION:

            switch (intersection_phase)
            {
                case LEFT_BRANCH_OUT:

                    if (data != DEAD_END)
                    {
                        branch_line_seen = 1;
                    }

                    if ((elapsed >= MIN_BRANCH_MS) &&
                        branch_line_seen &&
                        Dead_End_Confirmed(data))
                    {
                        intersection_phase = LEFT_BRANCH_BACK;

                        Start_Turn(TURN_LEFT_180);
                    }
                    else
                    {
                        Follow_Track(data);
                    }
                    break;

                case LEFT_BRANCH_BACK:

                    if ((elapsed >= MIN_BRANCH_MS) &&
                        (Is_T_Intersection(data) ||
                         Is_Right_Corner(data)))
                    {
                        intersection_phase = RIGHT_BRANCH_OUT;
                        phase_start_ms = Timer_A1_ms_elapsed;
                        dead_end_count = 0;
                        branch_line_seen = 0;

                        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
                    }
                    else
                    {
                        Follow_Track(data);
                    }
                    break;

                case RIGHT_BRANCH_OUT:

                    if (elapsed < CROSS_INTERSECTION_MS)
                    {
                        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
                    }
                    else
                    {
                        if (data != DEAD_END)
                        {
                            branch_line_seen = 1;
                        }

                        if ((elapsed >= MIN_BRANCH_MS) &&
                            branch_line_seen &&
                            Dead_End_Confirmed(data))
                        {
                            intersection_phase = RIGHT_BRANCH_BACK;

                            Start_Turn(TURN_LEFT_180);
                        }
                        else
                        {
                            Follow_Track(data);
                        }
                    }
                    break;

                case RIGHT_BRANCH_BACK:

                    if ((elapsed >= MIN_BRANCH_MS) &&
                        (Is_T_Intersection(data) ||
                         Is_Left_Corner(data)))
                    {
                        navigation_state = RETURN_TO_START;

                        Start_Turn(TURN_LEFT_90);
                    }
                    else
                    {
                        Follow_Track(data);
                    }
                    break;

                default:
                    navigation_error = 1;
                    navigation_state = FINISHED;
                    Motor_Stop();
                    break;
            }
            break;

        case RETURN_TO_START:

            if ((elapsed >= MIN_RETURN_MS) &&
                branch_line_seen &&
                Dead_End_Confirmed(data))
            {
                navigation_state = FINISHED;
                Motor_Stop();
            }
            else
            {
                Follow_Track(data);
            }
            break;

        case FINISHED:
        default:
            Motor_Stop();
            break;
    }
}

/**
 * Called every 1 ms by Timer_A1. Starts a sensor reading every 10 ms,
 * completes it 1 ms later, then updates the LEDs and maze controller.
 */
void Timer_A1_Periodic_Task(void)
{
    Timer_A1_ms_elapsed++;

    if ((Timer_A1_ms_elapsed % 10u) == 0u)
    {
        Reflectance_Sensor_Start();
    }

    if (((Timer_A1_ms_elapsed % 10u) == 1u) &&
        (Timer_A1_ms_elapsed > 10u))
    {
        uint8_t data = Reflectance_Sensor_End();

        Detect_Line_Position(data);

        Navigation_Controller(data);
    }
}

/**

 * Disables interrupts during setup, initializes the clock, onboard LEDs,
 * UART, reflectance sensor, motors, and timers, then enables interrupts.
 * @return Does not return during normal operation.
 */
int main(void)
{
    DisableInterrupts();

    Clock_Init48MHz();

    LED1_Init();
    LED2_Init();

    EUSCI_A0_UART_Init_Printf();

    Reflectance_Sensor_Init();

    Motor_Init();

    SysTick_Interrupt_Init(
        SYSTICK_INT_NUM_CLK_CYCLES,
        SYSTICK_INT_PRIORITY
    );

    Timer_A1_Interrupt_Init(
        &Timer_A1_Periodic_Task,
        TIMER_A1_INT_CCR0_VALUE
    );

    EnableInterrupts();

    while (1)
    {

    }
}
