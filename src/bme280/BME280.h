#pragma once

#include "common/LinuxI2CBus.h"
#include "common/status.h"

class BME280_Interface {
    public:
        explicit BME280_Interface(I2CBus *i2c, uint8_t address = 0x60);
        

    private:
        I2CBus *i2c_ = nullptr;
        uint8_t addr_ = 0x60;
};