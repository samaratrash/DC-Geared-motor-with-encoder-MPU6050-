# DC Motor Controller with Encoder and MPU6050 Integration

This project demonstrates interfacing a DC geared motor with an encoder and an MPU6050 accelerometer/gyroscope module using an Arduino microcontroller. The goal is to control the motor's rotation based on tilt angles and display real-time data about motor position and movement.

---

## Table of Contents
- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Wiring Setup](#wiring-setup)
- [Usage](#usage)
- [Code Structure](#code-structure)

---

## Project Overview

This project integrates a **DC motor with an encoder** and an **MPU6050 module** for real-time motor control. The MPU6050 detects tilt angles, and the motor responds accordingly by rotating forward or backward. The encoder provides precise feedback on motor position, allowing for comparison and validation of the MPU6050 data. The results are displayed on an LCD screen and printed to the serial monitor.

---

## Features

- **Motor Control**: Adjust motor speed and direction based on MPU6050 tilt angles.
- **Encoder Feedback**: Track motor rotations with high precision.
- **LCD Display**: View tilt angles, encoder ticks, and movement details in real-time.
- **Serial Output**: Debug and monitor the data flow through the serial interface.

---

## Hardware Requirements

- DC Geared Motor (N20) with Encoder
- L293D Motor Driver
- MPU6050 Accelerometer/Gyroscope Module
- 16x2 LCD Display
- Arduino Board (e.g., Uno, Mega, Nano)
- Jumper Wires
- External Power Supply for Motor (if required)

---

## Software Requirements

- Arduino IDE (Version 1.8.0 or newer)
- [Adafruit_ADXL345_U Library](https://github.com/adafruit/Adafruit_ADXL345)
- [LiquidCrystal Library](https://www.arduino.cc/en/Reference/LiquidCrystal)

---

## Wiring Setup

### Motor and Encoder
| Component Pin   | Arduino Pin |
|------------------|-------------|
| Motor In1        | 10          |
| Motor In2        | 13          |
| Motor Enable     | 9 (PWM)     |
| Encoder C1       | 2 (Interrupt Pin) |
| Encoder C2       | 3           |

### MPU6050
| MPU6050 Pin | Arduino Pin |
|-------------|-------------|
| VCC         | 5V          |
| GND         | GND         |
| SCL         | A4          |
| SDA         | A5          |

### LCD Display
| LCD Pin      | Arduino Pin |
|--------------|-------------|
| RS           | 12          |
| Enable       | 11          |
| D4           | 6           |
| D5           | 4           |
| D6           | 8           |
| D7           | 7           |

---

## Usage

1. Connect the hardware components as per the wiring setup.
2. Open the Arduino IDE and upload the provided sketch (`DCmotor.h`) to the Arduino.
3. Open the Serial Monitor at a baud rate of 9600 to view debug logs.
4. Tilt the MPU6050 module to observe motor rotation and encoder feedback displayed on the LCD.

---

## Code Structure

The code is organized as follows:

- **Setup**:
  - Initializes motor, encoder, LCD, and MPU6050.
  - Attaches interrupts for encoder tracking.
  
- **Main Loop**:
  - Reads accelerometer values from the MPU6050.
  - Calculates tilt angle and relative movement.
  - Controls motor direction and speed based on tilt.

- **Helper Functions**:
  - `moveMotor(float angle, bool forward)`: Moves the motor based on the calculated angle.
  - `stopMotor()`: Stops the motor.
  - `encoderISR()`: Handles encoder interrupts for tracking ticks.


