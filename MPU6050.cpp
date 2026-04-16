#include "headers/MPU6050.h"

#include <vector>
#include <cstdint>

MPU6050_Interface::MPU6050_Interface(LinuxI2c* i2c, uint8_t address)
    : i2c_(i2c), addr_(address) {}


bool MPU6050_Interface::Initialize() {
    if (!i2c_) {
        return false;
    }

    const uint8_t PWR_MGMT_1 = 0x6B;

    // Wake device: clear SLEEP bit (bit 6).
    if (!i2c_->WriteBit(addr_, PWR_MGMT_1, 6, 0)) {
        return false;
    }

    // Set clock source to PLL with X-axis gyroscope (CLKSEL bits 2:0 = 0b001).
    const uint8_t clock_source_ppl = 0x01;

    if (!i2c_->WriteField(addr_, PWR_MGMT_1, 0, 3, clock_source_ppl)) {
        return false;
    }

    return true;
}


// WHO_AM_I    0x75    Read to verify device (returns 0x68)
uint8_t MPU6050_Interface::WHO_AM_I() {
    const uint8_t error_code = 0xFF;

    if (!i2c_) {
        return error_code;
    }

    const uint8_t WHO_AM_I = 0x75;
    std::vector<uint8_t> output;

    if (!i2c_->ReadBlock(addr_, WHO_AM_I, 1, &output)) {
        return error_code;
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

    data.ax = raw.ax / accel_scale_;
    data.ay = raw.ay / accel_scale_;
    data.az = raw.az / accel_scale_;

    data.temp = (raw.temp / 340.0f) + 36.53f;

    data.gx = raw.gx / gyro_scale_;
    data.gy = raw.gy / gyro_scale_;
    data.gz = raw.gz / gyro_scale_;

    return data;
}

// callable read
IMU_Data MPU6050_Interface::Read() {
    IMU_Raw raw = ReadRaw();
    return Scale(raw);
}


/**
SetAccelRange() — Change accelerometer sensitivity. 
Writes to ACCEL_CONFIG (0x1C), specifically bits 4:3. 
The options are ±2g, ±4g, ±8g, ±16g. 
A wider range means you can measure bigger forces but with less precision. 
At ±2g you divide raw values by 16384 to get g's. At ±16g you divide by 2048.
*/

bool MPU6050_Interface::SetAccelRange(const int setting) {
    if (setting < 0 || setting > 3) {
        return false;
    }

    const uint8_t ACCEL_CONFIG = 0x1C;

    // AFS_SEL is bits 4:3 of ACCEL_CONFIG. setting maps directly to 0–3.
    if (!i2c_->WriteField(addr_, ACCEL_CONFIG, 3, 2, static_cast<uint8_t>(setting))) {
        return false;
    }

    // Keep the scale divisor in sync: ±2g→16384, ±4g→8192, ±8g→4096, ±16g→2048.
    static constexpr float kAccelScales[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    accel_scale_ = kAccelScales[setting];

    return true;
}


/**SetGyroRange()
Writes to GYRO_CONFIG (0x1B), bits 4:3. 
Options are ±250 (0), ±500 (1), ±1000 (2), ±2000 (3) degrees per second.
*/
bool MPU6050_Interface::SetGyroRange(const int FS_SEL_SETTING){
    if (FS_SEL_SETTING < 0 || FS_SEL_SETTING > 3){
        return false;
    }

    // convert setting to FS_SEL key
    const uint8_t GYRO_CONFIG = 0x1B;

    if (!i2c_->WriteField(addr_, GYRO_CONFIG, 3, 2, static_cast<uint8_t>(FS_SEL_SETTING))){
        return false;
    }

    static constexpr float kGyroScales[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    gyro_scale_ = kGyroScales[FS_SEL_SETTING];
    return true;
}


/**
SetSampleRate() — Controls how often the chip takes a new measurement.
Writes to SMPLRT_DIV (0x19). The formula is rate = 1000 / (1 + value). 
So writing 0 gives 1kHz, writing 9 gives 100Hz, writing 99 gives 10Hz. 
Faster rate means more data but more I2C traffic.
*/

bool MPU6050_Interface::SetSampleRate(const int sample_rate){

    

    const uint8_t SMPLRT_DIV = 0x19;
}