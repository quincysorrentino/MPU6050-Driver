#include "headers/MPU6050.h"

#include <vector>
#include <cstdint>

MPU6050_Interface::MPU6050_Interface(LinuxI2c* i2c, uint8_t address)
    : i2c_(i2c), addr_(address) {}


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


bool MPU6050_Interface::Initialize() {
    if (!i2c_) {
        return false;
    }
  
    const uint8_t PWR_MGMT_1 = 0x6B;

    // Wake device by setting SLEEP bit 6 to 0.
    if (!i2c_->WriteBit(addr_, PWR_MGMT_1, 6, 0)) {
        return false;
    }

    // Clock source bits (2:0) = 0b001 (PLL with X-axis gyroscope).
    if (!i2c_->WriteBit(addr_, PWR_MGMT_1, 1, 0)) {
        return false;
    }

    if (!i2c_->WriteBit(addr_, PWR_MGMT_1, 2, 0)) {
        return false;
    }

    if (!i2c_->WriteBit(addr_, PWR_MGMT_1, 0, 1)) {
        return false;
    }

    return true;
}


// WHO_AM_I    0x75    Read to verify device (returns 0x68)
uint8_t MPU6050_Interface::WHO_AM_I() {
    if (!i2c_) {
        return 0xFF;
    }

    const uint8_t WHO_AM_I = 0x75;
    std::vector<uint8_t> output;

    if (!i2c_->ReadBlock(addr_, WHO_AM_I, 1, &output)) {
        return 0xFF;
    }

    return output[0];
}

// Raw read
IMU_Raw MPU6050_Interface::ReadRaw() {
    IMU_Raw data{};

    if (!i2c_) return data;

    const uint8_t ACCEL_XOUT_H = 0x3B;
    std::vector<uint8_t> out;

    if (!i2c_->ReadBlock(addr_, ACCEL_XOUT_H, 14, &out)){
        return data;
    }

    if (out.size() < 14){
        return data;
    } 

    auto combine = [](uint8_t hi, uint8_t lo) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };

    data.ax = combine(out[0],  out[1]);
    data.ay = combine(out[2],  out[3]);
    data.az = combine(out[4],  out[5]);
    data.temp = combine(out[6], out[7]);
    data.gx = combine(out[8],  out[9]);
    data.gy = combine(out[10], out[11]);
    data.gz = combine(out[12], out[13]);

    return data;
}

// Scale the data 
IMU_Data MPU6050_Interface::Scale(const IMU_Raw& raw) {
    IMU_Data data{};

    // ±2g
    const float accel_scale = 16384.0f; 

    // ±250 deg/s
    const float gyro_scale  = 131.0f;   

    data.ax = raw.ax / accel_scale;
    data.ay = raw.ay / accel_scale;
    data.az = raw.az / accel_scale;

    data.temp = (raw.temp / 340.0f) + 36.53f;

    data.gx = raw.gx / gyro_scale;
    data.gy = raw.gy / gyro_scale;
    data.gz = raw.gz / gyro_scale;

    return data;
}

// callable read
IMU_Data MPU6050_Interface::Read() {
    IMU_Raw raw = ReadRaw();
    return Scale(raw);
}