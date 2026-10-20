# ECE 528/L - Robotics and Embedded Systems with Lab
**CSU Northridge**

**Department of Electrical and Computer Engineering**

## Line Follower Lab
The Line Follower lab interfaces with the following:

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

### References

- ECE 528/L, Fall 2026, Lab 3: Line Follower, Section 4.7, pages 19–21.


https://www.geeksforgeeks.org/cpp/modulo-operator-in-c-cpp-with-examples/
* User LEDs of the TI MSP432 LaunchPad
* Pololu Gearmotor with Encoder - [Product Link](https://www.pololu.com/product/3675)
* 8-Channel QTRX Sensor Array - [Product Link](https://www.pololu.com/product/3672)
