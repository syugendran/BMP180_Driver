
#include "bmp180.h"
#include <Wire.h>
#include <math.h> 


// Write a single byte to a register
static bool i2c_write_reg(uint8_t reg, uint8_t value){
    Wire.beginTransmission(BMP180_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

//read a single byte from a register
static bool i2c_read_byte(uint8_t reg, uint8_t *out)
{
    Wire.beginTransmission(BMP180_I2C_ADDR);
    Wire.write(reg);
    if(Wire.endTransmission(false) != 0) return false; //false = RESTART not STOP

    Wire.requestFrom((uint8_t)BMP180_I2C_ADDR, (uint8_t)1);
    if(!Wire.available()) return false;
    *out = Wire.read();
    return true;
}

//Read N bytes starting from a register (auto-increments address)
static bool i2c_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len){
    Wire.beginTransmission(BMP180_I2C_ADDR);
    Wire.write(reg);
    if(Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)BMP180_I2C_ADDR, (uint8_t)len);
    if(Wire.available() < len)return false;
    for(uint8_t i = 0; i < len ; i++)
    {
        buf[i] = Wire.read();
    }
    return true;
}


//CALIBRATION LOAD 

static bool bmp180_load_calibration(bmp180_t *dev) {
    uint8_t buf[22];

    //read all 22 bytes in one brust 
    if(!i2c_read_bytes(BMP180_REG_CALIB_START, buf, 22)){
        Serial.println("[BMP180] ERROR: Cannot Read Calibration Data");
        return false;
    }

    #define CALIB_S16(i) ((int16_t)((buf[i] << 8) | buf[i+1]))
    #define CALIB_U16(i) ((uint16_t)((buf[i] << 8) | buf[i+1]))

    dev->calib.AC1 = CALIB_S16(0);
    dev->calib.AC2 = CALIB_S16(2);
    dev->calib.AC3 = CALIB_S16(4);
    dev->calib.AC4 = CALIB_U16(6);
    dev->calib.AC5 = CALIB_U16(8);
    dev->calib.AC6 = CALIB_U16(10);
    dev->calib._B1  = CALIB_S16(12);
    dev->calib.B2  = CALIB_S16(14);
    dev->calib.MB = CALIB_S16(16);
    dev->calib.MC = CALIB_S16(18);
    dev->calib.MD = CALIB_S16(20);

    uint16_t *words = (uint16_t *)&dev->calib;

    for(int i = 0; i < 11; i++){
        if(words[i] == 0x0000 || words[i] == 0xFFFF) {
            Serial.printf("[BMP180] ERROR: calib[%d] = 0x%04x - invalid!\n", i , words[i]);
            return false;
        }
    }
    Serial.printf("[BMP180] AC1=%d AC2=%d AC3=%d AC4=%u AC5=%u AC6=%u\n",
        dev->calib.AC1, dev->calib.AC2, dev->calib.AC3,
        dev->calib.AC4, dev->calib.AC5, dev->calib.AC6);
    Serial.printf("[BMP180] _B1=%d B2=%d MB=%d MC=%d MD=%d\n",
        dev->calib._B1, dev->calib.B2, dev->calib.MB,
        dev->calib.MC, dev->calib.MD);
    return true;
}


bool bmp180_init(bmp180_t *dev, bmp180_oss_t oss)
{
    dev->initialized = false;
    dev->oss         = oss;

    Wire.begin(21,22);
    Wire.setClock(100000);

    delay(20);

    uint8_t chip_id;

    if(!i2c_read_byte(BMP180_REG_CHIP_ID , &chip_id)) {
        Serial.println("[BMP180] ERROR: Failed to read chip ID");
        return false;
    }
    if(chip_id != BMP180_CHIP_ID) {
        Serial.printf("[BMP180] ERROR: Wrong Chip ID: 0x%02X (expected 0x%02X)\n", chip_id, BMP180_CHIP_ID);
        return false;
    }
    Serial.println("BMP180 Chip ID Verfied: 0X55 OK");

    if(!bmp180_load_calibration(dev)) return false;

    dev->initialized = true;
    Serial.println("[BMP180] Initialization Complete!");
    return true;
}


static bool bmp180_read_raw(uint8_t cmd, uint16_t wait_ms,
                            bool is_pressure, bmp180_oss_t oss,
                            int32_t *raw_out) {
    if(!i2c_write_reg(BMP180_REG_CTRL_MEAS , cmd)){
        Serial.println("[BMP180] ERROR: Failed to write control register");
        return false;
    }                
    delay(wait_ms);
    
    if(!is_pressure) {

        uint8_t buf[2];
        if(!i2c_read_bytes(BMP180_REG_OUT_MSG, buf , 2)) return false;
        *raw_out = ((int32_t)buf[0] << 8) | buf[1];
    } else {

        uint8_t buf[3];
        if(!i2c_read_bytes(BMP180_REG_OUT_MSG, buf , 3))return false;
        *raw_out = ((((int32_t)buf[0] << 16) |
                     ((int32_t)buf[1] << 8)  |
                     (int32_t)buf[2]) >> (8 - oss));
    }
    return true;
}

static int32_t _b5;

bool bmp180_read_temperature(bmp180_t *dev, float *temp_c)
{
    if(!dev->initialized) return false;

    int32_t UT; //Step 1: Read Raw uncompenstated temperature (UT)
    if(!bmp180_read_raw(BMP180_CMD_TEMP, 5, false, BMP180_OSS_ULTRA_LOW, &UT)) return false;

    int32_t X1, X2, B5, T;

    X1 = ((int32_t)(UT - dev->calib.AC6) * (dev->calib.AC5) >> 15);

    X2 = ((int32_t)dev->calib.MC << 11) / (X1 + dev->calib.MD);

    B5 = X1 + X2;
    _b5 = B5;

    T = (B5 + 8) >> 4;

    *temp_c = T / 10.0f; 
    return true;
}


//Pressure Calculation

bool bmp180_read_pressure(bmp180_t *dev, int32_t *pressure_pa) {

    if(!dev->initialized) return false;

    static const uint8_t OSS_CMD[] = {0x34, 0x74, 0xB4, 0xF4};
    static const uint8_t OSS_WAIT[] = {5 ,8 ,14, 26};

    int32_t UP;
    if(!bmp180_read_raw(OSS_CMD[dev->oss], OSS_WAIT[dev->oss],true, dev->oss, &UP)) return false;

    int32_t X1, X2, X3, B3 , B6, p;
    uint32_t B4, B7;
    B6 = _b5 - 4000;

    X1 = (dev->calib.B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (dev->calib.AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)dev->calib.AC1 * 4 + X3) << dev->oss) + 2) >> 2;

    //Second round
    X1 = (dev->calib.AC3 * B6) >> 13;
    X2 = (dev->calib._B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = dev->calib.AC4 * (uint32_t)(X3 + 32768) >> 15;
    B7 = ((uint32_t)UP - B3) * (50000 >> dev->oss);

    //Combine
    if (B7 < 0x80000000UL) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
 
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p  = p + ((X1 + X2 + 3791) >> 4);
 
    *pressure_pa = p;
    return true;
}

//Altitude from Pressure

float bmp180_altitude(int32_t pressure_pa, float sea_level_hpa)
{
    // Formula from datasheet Section 3.6:
    // altitude = 44330 * (1 - (p / p0)^(1/5.255))
    float p  = (float)pressure_pa / 100.0f;  // Pa → hPa
    return 44330.0f * (1.0f - powf(p / sea_level_hpa, 1.0f / 5.255f));
}

int32_t bmp180_sea_level(int32_t pressure_pa, float altitude_m) {
    // Formula: p0 = p / (1 - altitude/44330)^5.255
    float p = (float)pressure_pa / 100.0f;  // Pa → hPa
    float p0_hpa = p / powf(1.0f - altitude_m / 44330.0f, 5.255f);
    return (int32_t)(p0_hpa * 100.0f);  // hPa → Pa
}
