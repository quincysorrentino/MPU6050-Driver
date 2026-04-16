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
	IMU_Data Read();

	// setting: 0=±2g, 1=±4g, 2=±8g, 3=±16g
	bool SetAccelRange(int setting);

	// FS_SEL_SETTING: 0=±250d/s, 1=±500d/s, 2=±1000d/s, 3=±2000d/s
	bool SetGyroRange(int FS_SEL_SETTING);

 private:
	LinuxI2c* i2c_ = nullptr;
	uint8_t addr_ = 0x68;

	// Divisors matching the active AFS_SEL setting (default ±2g).
	float accel_scale_ = 16384.0f;

	// Divisor matching the active GYRO_FS_SEL setting (default ±250°/s).
	float gyro_scale_ = 131.0f;

	IMU_Raw ReadRaw();
	IMU_Data Scale(const IMU_Raw& raw);
};
