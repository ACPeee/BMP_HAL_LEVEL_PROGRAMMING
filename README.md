# BMP180 STM32 HAL Driver

This project implements a BMP180 barometric pressure sensor driver using STM32 HAL and I2C communication.

## Features

- Calibration register reading
- Temperature measurement
- Pressure measurement
- Altitude calculation using the barometric formula
- Configurable oversampling (OSS0–OSS3)

## Calculated Outputs

- Temperature (°C)
- Pressure (Pa)
- Altitude (m)

## Hardware

- STM32 (tested with STM32F4)
- BMP180 barometric pressure sensor
- I2C communication

## Example Usage

```c
BMP_Calculate();
BMP_Config(oss3);

while(1)
{
    ReadAll();

    float temp = temperature;
    float press = press.p;
    float alt = altitude;
}
