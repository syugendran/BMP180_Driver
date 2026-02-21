

#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>
#include <stdbool.h>

#define BMP180_I2C_ADDR         0X77 //7-bit address
#define BMP180_CHIP_ID          0X55 //Expected Value

#define BMP180_REG_CHIP_ID      0XD0
#define BMP180_REG_SOFT_RESET   0XE0
#define BMP180_REG_CTRL_MEAS    0XF4
#define BMP180_REG_OUT_MSG      0XF6
#define BMP180_REG_OUT_LSB      0XF7
#define BMP180_REG_OUT_XLSB     0XF8
#define BMP180_REG_CALIB_START  0XAA

#define BMP180_CMD_TEMP         0X2E//Start temperature measurment 
#define BMP180_CMD_PRESS_OSS0   0X34//Pressure: Ultra Low Power
#define BMP180_CMD_PRESS_OSS1   0X74//Pressure :standard
#define BMP180_CMD_PRESS_OSS2   0XB4//Pressure:high resolution
#define BMP180_CMD_PRESS_OSS3   0XF4//Pressure: Ultra high resolution

typedef enum {
    BMP180_OSS_ULTRA_LOW  = 0, //4.5ms , 3uA
    BMP180_OSS_STANDARD   = 1, //7.5ms , 5uA
    BMP180_OSS_HIGH_RES   = 2, //13.5ms, 7uA
    BMP180_OSS_ULTRA_HIGH = 3, // 25.5ms , 12uA
}bmp180_oss_t;

typedef struct {
    int16_t AC1 , AC2 , AC3;
    uint16_t AC4 , AC5 , AC6;
    int16_t _B1 , B2;
    int16_t MB , MC , MD;
}bmp180_calib_t;

typedef struct 
{
    bmp180_calib_t calib;
    bmp180_oss_t   oss;
    bool           initialized;
}bmp180_t;

bool bmp180_init(bmp180_t *dev, bmp180_oss_t oss);
bool bmp180_read_temperature(bmp180_t *dev , float *temp_c);
bool bmp180_read_pressure(bmp180_t *dev , int32_t *pressure_pa);
float bmp180_altitude(int32_t pressure_pa, float sea_level_hpa);
int32_t bmp180_sea_level(int32_t pressure_pa, float altitude_m);

#endif
