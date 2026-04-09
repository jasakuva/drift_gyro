# Getting Started Guide

## Prerequisites
- ESP32 development board
- MPU6050 6-axis IMU sensor
- Arduino IDE installed
- Required libraries: ESP32Servo, Adafruit_MPU6050, Adafruit_Sensor

## Installation

### Clone Repository
git clone https://github.com/jasakuva/drift_gyro.git

### Install Libraries
Search and install in Arduino IDE:
- ESP32Servo
- Adafruit MPU6050
- Adafruit Sensor

### Hardware Connections
MPU6050 to ESP32:
- SDA → D4
- SCL → D5
- GND → GND
- VCC → 3.3V

RC/Servo connections:
- Steering Input → D0
- Gain Input → D1
- Servo Output → D3

### Upload
Open src/rc_drift_gyro_01.ino in Arduino IDE and upload to ESP32.

## First Run

1. Keep device stationary for gyro calibration
2. Open Serial Monitor at 115200 baud
3. System will output calibration status
4. Test by gently moving RC car

## Configuration

Parameters can be tuned through the settings system. Key parameters include gain values (0.5-2.0 range) and filter cutoff frequencies.

## Documentation

- See TECHNICAL.md for system architecture
- See API.md for API reference
- See README.md for project overview

## Support

For issues, check GitHub issues or review code comments.