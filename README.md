# ECE 528/L - Robotics and Embedded Systems with Lab
**CSU Northridge**

**Department of Electrical and Computer Engineering**

## Overview
The purpose of the Line Follower lab is to develop an autonomous robot using the TI-RSLK MAX and MSP432P401R LaunchPad. The robot uses the QTRX Reflectance Sensor Array to detect a black line on a white surface and control the DC motors to follow the line. The main objective is to implement a line-following controller that allows the robot to navigate through a maze, make steering corrections, explore a T-section, return to the starting point and reach the dead end. A finite state machine (FSM) can be used to manage the robot's navigation and decision making.
## Components Used

- TI MSP432P401R LaunchPad
- TI-RSLK MAX Robot Chassis and Motor Board
- 8-Channel QTRX Reflectance Sensor Array 
- DC Motors and Motor Drivers
- User LEDs of the TI MSP432 LaunchPad
- Six AA Batteries
- USB to Micro-USB Cable
- Black Electrical Tape and White Paper

## Pre-Lab Assignment

### Question 1: Chassis Board LED Initialization

Implement a `void` function named `Chassis_Board_LEDs_Init()` that takes no arguments and initializes the following pins as output GPIO pins. Initialize all four output values to zero.

| LED | GPIO Pin | Bitmask |
|---|---|---|
| Front Left Yellow LED | P8.0 | `0x01` |
| Front Right Yellow LED | P8.5 | `0x20` |
| Back Left Red LED | P8.6 | `0x40` |
| Back Right Red LED | P8.7 | `0x80` |

#### Implementation

```c
void Chassis_Board_LEDs_Init(void)
{
    P8->SEL0 &= ~(0xE1);
    P8->SEL1 &= ~(0xE1);
    P8->DIR |= 0xE1;
    P8->OUT &= ~(0xE1);
}
```

## Implementation

### System Initialization

The `main()` function initializes the MSP432P401R microcontroller and the hardware required for the Line Follower robot. Interrupts are initially disabled while the 48 MHz system clock, onboard LEDs, UART module, reflectance sensor, and DC motors are initialized.

The SysTick timer and Timer_A1 are then configured to generate periodic interrupts. After initialization is complete, interrupts are enabled so the robot can begin reading the reflectance sensor and controlling the motors.

### Line Position Detection

The `Detect_Line_Position()` function identifies the robot's position relative to the black line using the 8-bit readings from the reflectance sensor. The function uses a switch-case statement to match the nine predefined sensor patterns from the lab guide. Each pattern corresponds to a specific position for maneuvering through the maze. The onboard red LED and RGB LED display the corresponding colors specified in the lab's line-position detection table. If the reading does not match a predefined pattern, the LEDs are off.

### Periodic Sensor Reading

The `Timer_A1_Periodic_Task()` function controls when the reflectance sensor is sampled using periodic interrupts. Timer_A1 generates an interrupt every 1 ms. The function uses a counter and the modulo operator to initiate sensor sampling every 10 ms by calling `Reflectance_Sensor_Start()`.

One millisecond later, the function calls `Reflectance_Sensor_End()` to obtain the sensor reading. The reading is then passed to `Detect_Line_Position()` and `Navigation_Controller()` to update the onboard LEDs and control the robot's movement.

### Motor Control

The `Line_Follower_Controller()` function controls the DC motors based on the reflectance sensor readings. The controller uses `Reflectance_Sensor_Position()` to calculate the robot's position relative to the black line. When the robot moves away from the center of the line, the controller adjusts the motor PWM values to correct its direction. Larger deviations cause the robot to rotate toward the line using `Motor_Left()` or `Motor_Right()`. The controller also stops the motors when no black line is detected.

### Maze Navigation

The `Navigation_Controller()` function uses a finite state machine (FSM) to manage the robot's movement through the maze. The FSM consists of four main navigation states: `EXPLORE_MAZE`, `EXPLORE_INTERSECTION`, `RETURN_TO_START`, and `FINISHED`.

During `EXPLORE_MAZE`, the robot follows the black line until it detects a T-intersection. It then enters `EXPLORE_INTERSECTION`, where it navigates the left branch, returns to the intersection, explores the right branch, and returns.

Once both branches have been explored, the robot transitions to `RETURN_TO_START` and follows the original path back toward the starting point. When it reaches the final dead end, the FSM transitions to `FINISHED`.

### Main Program

The `main()` function performs the initial hardware configuration and enables the periodic interrupts required for the operation of the robot.

Unlike the previous labs, the main `while(1)` loop is empty. The functionality of this lab is handled through sensor readings, LED outputs, motor control, and FSMs.

---

## Analysis and Results

### Line Position Detection

The reflectance sensor was tested using black electrical tape on white printing paper. The sensor successfully detected the black line and provided readings corresponding to the robot's position.

### Motor Control and Steering

The Line Follower robot successfully followed the black electrical tape on the white surface. When the robot was centered on the line, it moved forward.

When the robot was not in the center, the motor controller adjusted the PWM values to correct its direction. These corrections allowed the robot to maintain its position relative to the line and continue to follow its path.

### Maze Navigation

The robot successfully navigated the line maze using the FSM. It followed the initial path, detected the T-intersection, and explored both branches.

After exploring the intersection, the robot returned along the path to the starting point and stopped at the dead end. The results demonstrated that the reflectance sensor, DC motor controller, periodic interrupt tasks, and FSM worked together to perform autonomous line following and maze navigation.

## Known Issues or Limitations

During initial testing, the robot did not consistently correct its position when moving slightly left or right of the path. The motor controller was adjusted by modifying the PWM values based on the sensor readings.

## Video Link
https://youtube.com/shorts/b7VD7udaSd8

## References

- ECE 528/L, Fall 2026, Lab 3: Line Follower, Section 4.7, pages 19–21.


https://www.geeksforgeeks.org/cpp/modulo-operator-in-c-cpp-with-examples/
* User LEDs of the TI MSP432 LaunchPad
* Pololu Gearmotor with Encoder (https://www.pololu.com/product/3675)
* 8-Channel QTRX Sensor Array (https://www.pololu.com/product/3672)
