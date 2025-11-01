# NovaAI Sensor Firmware

This repository contains the firmware for the NovaAI sensor board, which is built around the ESP32-S3 microcontroller. The firmware is designed to read data from various onboard sensors, process it, and transmit it over a serial connection.

## Project Overview

The main purpose of this firmware is to initialize and read data from connected sensors based on runtime configuration. It supports different modes of operation that control data rate, axis selection, and processing methods. The device is controlled via commands sent over the serial interface.

## Hardware

- **Microcontroller:** ESP32-S3 (specifically the `esp32s3box` board configuration in PlatformIO)
- **Sensors:**
    - **IIS3WDB:** High-frequency 3-axis digital accelerometer.
    - **ISM330DLC:** 3D accelerometer and 3D gyroscope.
    - **ADS1115:** Analog-to-Digital Converter.

## Software & Dependencies

This project is developed using the PlatformIO IDE with the Arduino framework.

- **Framework:** [Arduino](https://www.arduino.cc/)
- **IDE:** [PlatformIO](https://platformio.org/)
- **Core Libraries:**
    - `robtillaart/ADS1X15`: For communication with the ADS1115 ADC.
    - `bblanchon/ArduinoJson`: For JSON serialization/deserialization (though usage might vary).
- **Custom Libraries:**
    - `IIS3WDB_Func.h`: Wrapper for the high-frequency accelerometer.
    - `ADS_Func.h`: Wrapper for the ADC.
    - `ISM330DLC_Func.h`: Wrapper for the accelerometer/gyroscope.

## Getting Started

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    ```
2.  **Open in PlatformIO:**
    Open the `firmware/platfromio` directory in Visual Studio Code with the PlatformIO extension installed.
3.  **Build:**
    ```bash
    pio run
    ```
4.  **Upload:**
    Connect the ESP32-S3 board and use the PlatformIO upload command. The board should be automatically detected.
    ```bash
    pio run --target upload
    ```

## Serial Communication

The firmware is controlled by sending specific commands over the serial port (Monitor Speed: `115200`).

- **`start`**: Initializes data collection and transmission.
- **`reset`**: Restarts the ESP32 microcontroller.
- **`mode_1`**: Sets the device to send RMS/STD values.
- **`mode_2`**: Sets the device to send data at the full Output Data Rate (ODR) for all axes.
- **`mode_3`**: Sets the device to send data at half ODR for all axes.
- **`mode_4`**: Sets the device to send data at quarter ODR for all axes.

The active sensor can be configured in the `src/main.cpp` file by changing the boolean flags `High_Freq_acc_sensor`, `ADS_active`, or `ISM330_active`.
