# BMP180 Driver (ESP32)

A small and simple BMP180 (Bosch) driver written in C/C++ for Arduino-style projects.

This driver communicates over I2C using `Wire.h` and supports:

- Sensor initialization with chip ID verification (0x55)
- Automatic calibration data loading
- Temperature reading (°C)
- Pressure reading (Pa)
- Altitude calculation (meters)
- Sea-level pressure calculation

The goal was to keep it lightweight, readable, and easy to integrate into embedded projects.

---

## Files

- `bmp180.h` – Public API, structs, register definitions
- `bmp180.c` – Driver implementation

---

## Hardware Connection

BMP180 uses I2C (7-bit address: `0x77`).

### Default ESP32 Pins (used in this driver)

- SDA → GPIO 21  
- SCL → GPIO 22  

Power connections:

- VCC → 3.3V  
- GND → GND  

If you're using different I2C pins, modify this line in `bmp180_init()`:

```cpp
Wire.begin(21,22);