# Moving objects controller
## Description
Sends a (Steer, Speed) couple over UART.
These values are computed :
 * from 2 ADC, e.g. an analog joystick wired to ESP32.
 * from pitch/roll got from a SensortagBLE server over a BLE connection.

## Usage
From that you can control some [hacked hoverboards](https://github.com/NiklasFauth/hoverboard-firmware-hack)

## Folder structure
### `ctrl/steer_speed_ctrl`
Arduino sketch running on ESP32 targets.
It can either :
 * get roll, pitch over Bluetooth from a customizedCC2650. It acts as a Bluetooth client connecting.
 * get two axis analog joystick values
and converts these data to a (steer, speed) target sent over UART. It is sent as 2 int16_t in Little Endian.

It also displays some info about speed, control type on a led strip. It can adjust control from smootth to sport mode using a potentiometer.

### `cc2650_stk_fw`
Binary of cc2650 sensortag firmware, it must be flashed to a Sensortag.
When power button is pressed, ESP32 connects/disconnects to it.

### `hoverboard_firmware_hack`
Hoverboard hacked firmware to flash to an existing hoverboard.

## Wheel chair use case
A wheel chair can be controlled with a Joystick or over Bluetooth. People siting on it will have lot a fun to control it using joystick. They will be surprised when you will take chair control pressing sensortag power button and moving it!

<img src="https://raw.githubusercontent.com/Lahorde/steer_speed_ctrl/master/img/wheelchair.jpg" width="500">

