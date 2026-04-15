// Minimal MPU6050 interface definition.
#pragma once

#include <cstdint>
#include <vector>
#include "I2C.h"

struct IMU_Raw {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
	};

	struct IMU_Data {
		float ax, ay, az;
		float temp;
		float gx, gy, gz;
	};


class MPU6050_Interface {
 public:
	explicit MPU6050_Interface(LinuxI2c* i2c, uint8_t address = 0x68);
    bool Initialize();
	uint8_t WHO_AM_I() const;

	IMU_Raw ReadRaw();
	IMU_Data Scale(const IMU_Raw& raw);
	IMU_Data Read();

 private:
	LinuxI2c* i2c_ = nullptr;
	uint8_t addr_ = 0x68;
};
