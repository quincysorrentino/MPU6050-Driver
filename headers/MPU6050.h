// Minimal MPU6050 interface definition.
#pragma once

#include <cstdint>

#include "I2C.h"

class MPU6050_Interface {
 public:
	explicit MPU6050_Interface(LinuxI2c* i2c, uint8_t address = 0x68);
    bool Initialize();
	bool ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz);


 private:
	LinuxI2c* i2c_ = nullptr;
	uint8_t addr_ = 0x68;
};
