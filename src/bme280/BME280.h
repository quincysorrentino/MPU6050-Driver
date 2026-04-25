#pragma once

#include "common/LinuxI2CBus.h"
#include "common/status.h"
#include <cstdint>

struct BME280Calib {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1, dig_H3;
    int16_t  dig_H2, dig_H4, dig_H5;
    int8_t   dig_H6;
};

struct BME280Data {
    uint32_t raw_pressure;    // up[19:0]  from 0xF7–0xF9
    uint32_t raw_temperature; // ut[19:0]  from 0xFA–0xFC
    uint16_t raw_humidity;    // uh[15:0]  from 0xFD–0xFE
    int32_t  temperature;     // 0.01 °C  (e.g. 2345 = 23.45 °C)
    uint32_t pressure;        // Pa × 256  Q24.8  (divide by 256 for Pa)
    uint32_t humidity;        // %RH × 1024  Q22.10  (divide by 1024 for %RH)
};

class BME280_Interface {
    public:
        explicit BME280_Interface(I2CBus *i2c, uint8_t address = 0x76);
        DriverStatus Initialize();
        DriverStatus GET_ID(uint8_t* out);
        DriverStatus RESET();
        DriverStatus Read(BME280Data* out);

    private:
        DriverStatus LoadCalibration();
        int32_t  CompensateTemperature(int32_t adc_T, int32_t& t_fine);
        uint32_t CompensatePressure(int32_t adc_P, int32_t t_fine);
        uint32_t CompensateHumidity(int32_t adc_H, int32_t t_fine);

        I2CBus *i2c_ = nullptr;
        uint8_t addr_ = 0x76;
        bool initialized_ = false;
        BME280Calib calib_{};
};
