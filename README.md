# Light Source & Object Proximity Detector System

## Introduction
An embedded system based on the **MSP430 microcontroller** for detecting light sources and monitoring object proximity.  
The system integrates an ultrasonic sensor, an LDR light sensor, and a servo motor for environmental scanning.  
It uses a modular architecture (HAL, BSP, APP) and a **finite state machine (FSM)**.  
A dedicated **File Mode** provides a lightweight file system for calibration data, file storage, and browsing.

## Features
- **Object Detection**: Scans the environment to locate objects and measure distances.  
- **Telemeter Mode**: Provides real-time distance measurement at a specific angle.  
- **Light Detection**: Detects light intensity at different angles.  
- **Combined Detection**: Performs simultaneous light + distance scanning.  
- **File Mode**: Manages flash-based file system, stores calibration values, saves files, and allows browsing.  
- **PC Communication**: UART interface for logging, file transfer, and interaction.  

## Hardware Components
- MSP430 microcontroller  
- Ultrasonic sensor for distance measurement  
- LDR sensor + ADC for light detection  
- Servo motor for 0–180° scanning  
- Flash memory - file system and storage  

## Software Components
- **MCU Firmware (C)**: Implements the finite state machine (FSM), controls sensor acquisition (ultrasonic, LDR), servo motor scanning, and UART communication. Includes a flash-based file system module for calibration storage, file saving, and browsing.  
- **PC Interface (Python)**: Provides a user interface for initiating scans, logging results, transferring files, and interacting with the system over UART.  

