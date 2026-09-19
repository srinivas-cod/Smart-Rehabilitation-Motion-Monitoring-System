# Q-REHAB: QNX-Based Rehabilitation Robotic Arm Controller

Q-REHAB is a prototype rehabilitation robotic arm control system built using **QNX 8.0 on Raspberry Pi 4** and **Arduino UNO**. The Raspberry Pi acts as the master safety supervisor, while the Arduino controls the robotic arm actuators.

## Key Features

* QNX 8.0 real-time supervisory control
* Five-joint robotic arm monitoring
* UART communication at 115200 baud
* JSON-based Arduino feedback
* QNX native message passing (IPC)
* Multi-threaded architecture
* Real-time joint-limit validation
* Last-safe-position storage
* Software emergency-stop commands
* Peltier actuator safety timeout
* Periodic QNX timer-pulse watchdog
* Recovery command support

## System Architecture

* **Raspberry Pi 4 + QNX:** Safety supervision and coordination
* **Arduino UNO:** Servo and actuator control
* **UART:** Communication between QNX and Arduino
* **QNX IPC:** Internal thread communication
* **Safety Coordinator:** Feedback validation and safety decisions

## Safety Mechanisms

The system validates joint positions against predefined limits and monitors Arduino feedback availability. When an unsafe condition or feedback timeout is detected, the controller sends a STOP command and disables the Peltier actuator.

> **Note:** This is a research prototype. Software safety commands are not a replacement for an independent physical emergency-stop circuit or formal medical-device safety validation.

## Technologies

`C++` `QNX 8.0` `Raspberry Pi 4` `Arduino UNO` `UART` `QNX IPC` `POSIX Threads` `Real-Time Systems`

**Project Focus:** Embedded Systems • Real-Time Operating Systems • Robotics • Safety-Critical Control
