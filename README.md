## Synopsis
This project mainly for the STM32F103 family of micro-controllers (MCU), the firmware is written for 2 purpose:
1. Receive speed information from the SBC and generate PPM signal for controling brushless motors
2. Read IMU sensor,perform AHRS calculation using Madgwick algorithm and send it back to the SBC

## Prerequisites
1. Eclipse 
2. GCC tool chain (follow link in installation)
3. ST-Link v2 in-circuit debugger and programmer
4. Patience not to kill me for poor documentation

## Installation
We are using Eclipse to develop the firmware

The tool chain in use is GNU Tool for ARM Embedded Processors [arm-none-eabi-gcc]

A rough outline of the process flow:
1. Compile and build the project to generate the binaraies.
2. Connect up ST-LinkV2
3. Transfer code over via arm

Check out the link below for detailed steps on setuping the enviorment and flashing the MCU
https://www.overleaf.com/read/xykrnzhmsymq

## History
Version 0.1 Basic skeleton of firmware for STM32F103 Family only

## Authors
1. Louis Goh - Initial Work

## License
The MIT License (MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Acknowledgment
1. Kris Winer for his library on Madgwick Algorithm
2. Simpyl Embedded for a cool implementation of the ring buffer
3. Justin Wang for the i2c implementation
4. And to whoever I have missed out
