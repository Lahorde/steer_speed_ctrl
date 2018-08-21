# Moving objects controller
## Description
ESP32 project that sends a (Steer, Speed) couple over UART.
These values are computed :
 * from 2 ADC, e.g. an analog joystick wired to ESP32.
 * from pitch/roll got from a SensortagBLE server over a BLE connection.

It is sent as 2 int16_t in Little Endian.

## Usage
From that you can control some [hacked hoverboards](https://github.com/NiklasFauth/hoverboard-firmware-hack)