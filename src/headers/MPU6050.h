#pragma once

#include <cstdint>
#include <vector>
#include "I2C.h"
#include "status.h"

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

    DriverStatus Initialize();
    DriverStatus WHO_AM_I(uint8_t* out) const;
    DriverStatus Read(IMU_Data* out);

    // setting: 0=±2g, 1=±4g, 2=±8g, 3=±16g
    DriverStatus SetAccelRange(int setting);
    // FS_SEL_SETTING: 0=±250d/s, 1=±500d/s, 2=±1000d/s, 3=±2000d/s
    DriverStatus SetGyroRange(int FS_SEL_SETTING);
    DriverStatus SetSampleRate(int sample_rate);

	DriverStatus Reset();
    DriverStatus Sleep();
    DriverStatus Wake();

 private:
    LinuxI2c* i2c_ = nullptr;
    uint8_t addr_ = 0x68;
    bool initialized_ = false;

    float accel_scale_ = 16384.0f;
    float gyro_scale_  = 131.0f;
    float mpu_sample_rate_ = 8.0f;

    DriverStatus ReadRaw(IMU_Raw* out);
    IMU_Data Scale(const IMU_Raw& raw);
    DriverStatus calc_sample_rate(int sample_rate);
};
