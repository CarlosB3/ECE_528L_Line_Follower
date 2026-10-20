/**
 * @file main.c
 *
 * @brief Main source code for the Line_Follower program.
 *
 * This file contains the main entry point for the Line_Follower program.
 * The main controller demonstrates a Line Follower robot without using an advanced algorithm.
 *
 * It interfaces the following peripherals using GPIO to demonstrate line following:
 *  - 8-Channel QTRX Sensor Array module
 *
 * Timers are used in this lab:
 *  - SysTick:  Used to generate periodic interrupts at a specified rate (1 kHz)
 *  - Timer A0: Used to generate PWM signals that will be used to drive the DC motors
 *  - Timer A1: Used to generate periodic interrupts at a specified rate (1 kHz)
 *
 * @note For more information regarding the 8-Channel QTRX Sensor Array module,
 * refer to the product page: https://www.pololu.com/product/3672
 *
 * @author
 *
 */
#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/Timer_A1_Interrupt.h"
#include "inc/SysTick_Interrupt.h"
#include "inc/Reflectance_Sensor.h"

#define PWM_NOMINAL 2500

// Initialize a global variable for Timer A1 to keep track of elapsed time
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

void Detect_Line_Position(uint8_t reflectance_sensor_data)
{
    LED2_Output(0x00);
    LED1_Output(0x00);

    switch (reflectance_sensor_data)
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
            LED2_Output(0x07);
            LED1_Output(0x01);
            break;

        case RIGHT_TURN:
            LED2_Output(0x06);
            LED1_Output(0x01);
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


void Line_Follower_Controller(uint8_t reflectance_sensor_data)
{
    switch (reflectance_sensor_data)
    {
        case CENTER:
            Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
            break;

        case SLIGHT_LEFT_CENTER:
            // Small left correction
            break;

        case SLIGHT_RIGHT_CENTER:
            // Small right correction
            break;

        case FAR_LEFT:
            // Stronger left correction
            break;

        case FAR_RIGHT:
            // Stronger right correction
            break;

        case LEFT_TURN:
            // Handle left turn
            break;

        case RIGHT_TURN:
            // Handle right turn
            break;

        case T_INTERSECTION:
            // Explore branches and track progress
            break;

        case DEAD_END:
            // Handle end of line
            break;

        default:
            // Handle unexpected readings
            break;
    }
}

void Timer_A1_Periodic_Task(void)
{
    // Increment Timer A1 ms elapsed by 1 every time the Timer A1 periodic interrupt occurs
    Timer_A1_ms_elapsed++;

    // Start the process of reading the reflectance sensor array every 10 ms (i.e. 10, 20, 30, ...)
    if ((Timer_A1_ms_elapsed % 10) == 0)
    {
        Reflectance_Sensor_Start();
    }

    // Finish reading the reflectance sensor array after 1 ms (i.e. 11, 21, 31, ...)
    if ((Timer_A1_ms_elapsed % 10) == 1)
    {
        uint8_t Reflectance_Sensor_Data = Reflectance_Sensor_End();
        Detect_Line_Position(Reflectance_Sensor_Data);
        Line_Follower_Controller(Reflectance_Sensor_Data);
    }
}

int main(void)
{
    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED and the RGB LED on the MSP432 microcontroller
    LED1_Init();
    LED2_Init();

    // Initialize the EUSCI_A0 UART module
    EUSCI_A0_UART_Init_Printf();

    // Initialize the 8-Channel QTRX Reflectance Sensor Array module
    Reflectance_Sensor_Init();

    // Initialize the SysTick timer to generate periodic interrupts every 1 ms
    SysTick_Interrupt_Init(SYSTICK_INT_NUM_CLK_CYCLES, SYSTICK_INT_PRIORITY);

    // Initialize Timer A1 periodic interrupts every 1 ms
    Timer_A1_Interrupt_Init(&Timer_A1_Periodic_Task, TIMER_A1_INT_CCR0_VALUE);

    // Enable the interrupts used by the modules
    EnableInterrupts();

    while(1)
    {

    }
}
