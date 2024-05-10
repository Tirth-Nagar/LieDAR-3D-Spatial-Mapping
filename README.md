# TGN-6909 LieDAR - 3D Spatial Mapping

The TGN-6909 is a compact and cost-effective embedded spatial measurement system designed for indoor 3D mapping. It utilizes time-of-flight sensing technology and a rotary mechanism to provide precise 360-degree distance measurements within a single vertical plane, while integrating fixed distance samples along the orthogonal axis.

### Device Image

![Device_Image.png](https://github.com/Tirth-Nagar/LieDAR-3D-Spatial-Mapping/blob/2830078ba3c2d4bd4af3525be7b3637d44c96efa/Assets/Product.jpg)

## Features

- Built-in error checking and on-the-fly error mitigation
- Single-Button interrupt based operation
- 4 onboard status LEDs
- Variable precision with customizable number of measurements per 360° rotation
- Real-time data transmission and processing
- Asynchronous data transmission for high-speed applications
- Fully automated 3D Reconstruction

## Hardware Setup

### MSP432E401Y

| Parameter | Value |
|------------|-------|
| Clock Speed | 96 MHz |
| Baud Rate | 115200 |
| Processor | Cortex M4F |
| Serial Port | COM 4 |

### Digital I/O

| Component | Pin |
|------------|-----|
| LED 1 | PN1 |
| LED 2 | PN0 |
| LED 3 | PF4 |
| LED 4 | PF0 |
| Onboard Button | PJ1 |

### ULN2003 Driver Board

| ULN2003 Pin | Microcontroller |
|--------------|-----------------|
| IN_1 - IN_4 | PH0 - PH3 |
| V+ | 5V |
| V- | GND |

### VL53L1X

| VL53L1X | Microcontroller |
|----------|-----------------|
| Vin | 3.3V |
| GND | GND |
| SCL | PB2 |
| SDA | PB3 |

### Schematic

![Schematic.png](https://github.com/Tirth-Nagar/LieDAR-3D-Spatial-Mapping/blob/2830078ba3c2d4bd4af3525be7b3637d44c96efa/Assets/Schematic.png)

## Device Overview

### Features

#### Texas Instrument MSP-EXP432E401Y
- Facilitates seamless data exchange between TOF sensor and computer systems
- Employs a powerful 32-bit Cortex M4F CPU for efficient processing
- Operates at 96 MHz bus speed for swift processing
- Offers ample storage with 256 KB SRAM and 1 MB Flash Memory
- Enhances versatility with General Purpose Input/Output (GPIO) Pins
- Provides clear visual feedback and status indication through integrated LEDs

#### VL53L1X Time of Flight Sensor Capabilities
- Capable of measuring distances up to 4 m
- Operates efficiently at 50 Hz frequency for rapid data acquisition and processing
- Offers flexibility with 2.6 V to 3.5 V operating voltage range

#### 28BYJ-48 Stepper Motor and ULN2003 Diver Board Features
- Moves precisely with 4 phases per step for accurate spatial measurements
- Provides fine 512 steps per 360-degree rotation granularity for precise positioning
- Features 4 LEDs on the driver board for clear phase indication and monitoring
- Offers 5 V to 12 V operating voltage range flexibility

#### Serial Communication Protocols
- Facilitates seamless data collection via I2C from sensor to microcontroller
- Utilizes UART protocol at 115200 baud rate for rapid data transmission to PC

#### Environment Recreation
- Enables serial communication with PC via Pyserial library in Python
- Utilizes Open3D library in Python for 3D visualization post-processing

## General Description

This device seamlessly integrates an onboard Time-of-Flight (ToF) sensor for efficient 3D visualization. It captures data across the xy-plane at 11.25-degree intervals, with LED 1 momentarily lighting up upon successful distance measurement before a brief pause for data processing.

Operation is initiated by pressing the microcontroller's reset button and toggling the motor using the onboard PJ1 button, followed by executing a designated Python script. Upon system startup and ToF sensor activation, LEDs 1-4 illuminate, signifying operational readiness.

The sensor transmits data to the board via I2C, which is then relayed to an external PC through UART for real-time processing and visualization. The MSPEXP432E401Y microcontroller orchestrates timing with precision, operating at 96 MHz bus speed.

Distance measurements are calculated by the ToF sensor using the formula: half the photon travel time multiplied by the speed of light.

Data processing and visualization are executed using Python and the Open3D library, transforming polar data into Cartesian coordinates through trigonometric calculations. The processed data is then plotted on a 3D Cartesian axis, offering a comprehensive visualization of the scanned area.

### Block Diagram

![Block-Diagram.png](https://github.com/Tirth-Nagar/LieDAR-3D-Spatial-Mapping/blob/2830078ba3c2d4bd4af3525be7b3637d44c96efa/Assets/Block%20Diagram.png)

### Results

15-Layer scan of the Hallway using Open3D

![Angled_Scan.png](https://github.com/Tirth-Nagar/LieDAR-3D-Spatial-Mapping/blob/2830078ba3c2d4bd4af3525be7b3637d44c96efa/Assets/Angled.png) ![Front_Scan.png](https://github.com/Tirth-Nagar/LieDAR-3D-Spatial-Mapping/blob/2830078ba3c2d4bd4af3525be7b3637d44c96efa/Assets/Front.png)

Hallway in question:

![Hallway.png](https://github.com/Tirth-Nagar/LieDAR-3D-Spatial-Mapping/blob/2830078ba3c2d4bd4af3525be7b3637d44c96efa/Assets/Hallway.jpg)


