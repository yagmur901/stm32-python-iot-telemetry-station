# IoT Telemetry Station: STM32 & Python Integration

## Overview
This project demonstrates a comprehensive end-to-end hardware and software integration for a real-time, multi-sensor IoT data acquisition system. Utilizing an STM32 microcontroller, the system continuously reads environmental and physical data from various analog and digital sensors. 

The raw telemetry data is securely packaged into a custom binary struct using the Consistent Overhead Byte Stuffing (COBS) algorithm and an XOR checksum, then transmitted over a UART interface. A Python middleware decodes this continuous data stream, logs the telemetry, and executes OS-level security protocols based on specific hardware triggers.

## Key Features
* **Multi-Sensor Data Fusion:** Seamlessly integrates multiple communication protocols and peripherals on the STM32:
  * **ADC:** Reading Light Intensity (LDR), Water Level (HW-038), and Soil Moisture.
  * **Timers & PWM:** Controlling Servo Motor angles and accurately calculating Ultrasonic sensor distances.
  * **Microsecond Timing:** Utilizing hardware-level interrupt disabling for precise DHT11 temperature and humidity acquisition.
* **Data Integrity Protocol:** Implements the COBS algorithm combined with a calculated XOR checksum across a 13-variable data packet (`<IBBBBHHHHBBBB`).
* **OS-Level Automation:** Python middleware capable of detecting specific hardware states (Lock_Flag) to trigger OS commands like locking the workstation via `user32.dll`.
