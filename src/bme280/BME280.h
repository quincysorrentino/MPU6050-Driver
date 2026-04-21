#pragma once

#include "common/LinuxI2CBus.h"
#include "common/status.h"

class BME280_Interface {
    public:
        explicit BME280_Interface(I2CBus *i2c, uint8_t address = 0x76);
        DriverStatus Initialize();
        DriverStatus GET_ID(uint8_t* out);
        DriverStatus RESET();

    private:
        I2CBus *i2c_ = nullptr;
        uint8_t addr_ = 0x76;
        bool initialized_ = false;
        uint8_t temp_trim_ = 0;
        uint8_t pressure_trim_ = 0;
        uint8_t humidity_trim_ = 0;
};

