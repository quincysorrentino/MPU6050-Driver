#include "headers/MPU6050.h"

#include <vector>

MPU6050_Interface::MPU6050_Interface(LinuxI2c* i2c, uint8_t address)
    : i2c_(i2c), addr_(address) {}

bool MPU6050_Interface::Initialize() {
    if (!i2c_) {
        return false;
    }
    
    // wake up MPU6050 (write 0x00 to 0x6B)
    const uint8_t PWR_MGMT_1 = 0x6B;

    uint8_t wake_value = 0x00;
    if (!i2c_->WriteByte(addr_, PWR_MGMT_1, wake_value)) {
        return false;
    }

    // set clock source (write 0x01 to 0x6B)
    uint8_t clock_source_value = 0x01;
    if (!i2c_->WriteByte(addr_, PWR_MGMT_1, clock_source_value)) {
        return false;
    }

    return true;
}

bool MPU6050_Interface::ReadGyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    if (!i2c_ || !gx || !gy || !gz) {
        return false;
    }

    // Gyro X/Y/Z registers start at 0x43.
    const uint8_t GYRO_XOUT_H = 0x43;
    std::vector<uint8_t> buf;
    if (!i2c_->ReadBlock(addr_, GYRO_XOUT_H, 6, &buf)) {
        return false;
    }

    *gx = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    *gy = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    *gz = static_cast<int16_t>((buf[4] << 8) | buf[5]);
    return true;
}
