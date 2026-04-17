#include "headers/MPU6050.h"

#include <vector>
#include <cstdint>

MPU6050_Interface::MPU6050_Interface(LinuxI2c *i2c, uint8_t address)
    : i2c_(i2c), addr_(address) {}

/**
 * Initialize() — Wake the chip and verify identity.
 * Writes 0x00 to PWR_MGMT_1 (0x6B) to clear the SLEEP bit, then reads WHO_AM_I (0x75) to confirm you're talking to a real MPU6050.
 * You should call this once before anything else.
 */
DriverStatus MPU6050_Interface::Initialize()
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    const uint8_t PWR_MGMT_1 = 0x6B;

    if (!i2c_->WriteBit(addr_, PWR_MGMT_1, 6, 0))
        return DriverStatus::ERR_I2C_WRITE;

    const uint8_t clock_source_ppl = 0x01;
    if (!i2c_->WriteField(addr_, PWR_MGMT_1, 0, 3, clock_source_ppl))
        return DriverStatus::ERR_I2C_WRITE;

    return DriverStatus::OK;
}

DriverStatus MPU6050_Interface::WHO_AM_I(uint8_t *out) const
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    const uint8_t WHO_AM_I_REG = 0x75;
    std::vector<uint8_t> buf;

    if (!i2c_->ReadBlock(addr_, WHO_AM_I_REG, 1, &buf))
        return DriverStatus::ERR_I2C_READ;

    *out = buf[0];
    return DriverStatus::OK;
}

DriverStatus MPU6050_Interface::ReadRaw(IMU_Raw *out)
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    const uint8_t ACCEL_XOUT_H = 0x3B;
    std::vector<uint8_t> buf;

    if (!i2c_->ReadBlock(addr_, ACCEL_XOUT_H, 14, &buf))
        return DriverStatus::ERR_I2C_READ;

    if (buf.size() < 14)
        return DriverStatus::ERR_BAD_DATA;

    auto combine = [](uint8_t hi, uint8_t lo) -> int16_t
    {
        return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };

    out->ax = combine(buf[0], buf[1]);
    out->ay = combine(buf[2], buf[3]);
    out->az = combine(buf[4], buf[5]);
    out->temp = combine(buf[6], buf[7]);
    out->gx = combine(buf[8], buf[9]);
    out->gy = combine(buf[10], buf[11]);
    out->gz = combine(buf[12], buf[13]);

    return DriverStatus::OK;
}

IMU_Data MPU6050_Interface::Scale(const IMU_Raw &raw)
{
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

/**
 * Read() — Get all sensor data in one shot.
 * Reads 14 bytes starting at ACCEL_XOUT_H (0x3B).
 * Those 14 bytes are packed in order: accel X/Y/Z (6 bytes), temperature (2 bytes), gyro X/Y/Z (6 bytes).
 * Each value is two bytes, big-endian, signed.
 * You combine them, then divide by the scale factor to get real units.
 */
DriverStatus MPU6050_Interface::Read(IMU_Data *out)
{
    IMU_Raw raw{};
    DriverStatus s = ReadRaw(&raw);
    if (s != DriverStatus::OK)
        return s;
    *out = Scale(raw);
    return DriverStatus::OK;
}

/**
 * SetAccelRange() — Change accelerometer sensitivity.
 * Writes to ACCEL_CONFIG (0x1C), specifically bits 4:3.
 * The options are ±2g, ±4g, ±8g, ±16g. A wider range means you can measure bigger forces but with less precision.
 * At ±2g you divide raw values by 16384 to get g's. At ±16g you divide by 2048.
 */
DriverStatus MPU6050_Interface::SetAccelRange(const int setting)
{
    if (setting < 0 || setting > 3)
        return DriverStatus::ERR_BAD_PARAM;

    const uint8_t ACCEL_CONFIG = 0x1C;
    if (!i2c_->WriteField(addr_, ACCEL_CONFIG, 3, 2, static_cast<uint8_t>(setting)))
        return DriverStatus::ERR_I2C_WRITE;

    static constexpr float kAccelScales[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    accel_scale_ = kAccelScales[setting];
    return DriverStatus::OK;
}

/**
 * SetGyroRange() — Same idea for the gyroscope.
 * Writes to GYRO_CONFIG (0x1B), bits 4:3. Options are ±250, ±500, ±1000, ±2000 degrees per second.
 * A slow-spinning thing like a phone needs ±250. A fast-spinning drone motor needs ±2000.
 */
DriverStatus MPU6050_Interface::SetGyroRange(const int FS_SEL_SETTING)
{
    if (FS_SEL_SETTING < 0 || FS_SEL_SETTING > 3)
        return DriverStatus::ERR_BAD_PARAM;

    const uint8_t GYRO_CONFIG = 0x1B;
    if (!i2c_->WriteField(addr_, GYRO_CONFIG, 3, 2, static_cast<uint8_t>(FS_SEL_SETTING)))
        return DriverStatus::ERR_I2C_WRITE;

    static constexpr float kGyroScales[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    gyro_scale_ = kGyroScales[FS_SEL_SETTING];
    return DriverStatus::OK;
}

DriverStatus MPU6050_Interface::calc_sample_rate(const int sample_rate)
{
    const uint8_t CONFIG = 0x1A;
    uint8_t DLPF_CFG_VAL;

    if (!i2c_->ReadField(addr_, CONFIG, 0, 2, &DLPF_CFG_VAL))
        return DriverStatus::ERR_I2C_READ;

    uint8_t gyro_output_rate = (DLPF_CFG_VAL != 0) ? 1 : 8;
    mpu_sample_rate_ = gyro_output_rate / (1 + sample_rate);
    return DriverStatus::OK;
}

/**
 * SetSampleRate() — Controls how often the chip takes a new measurement.
 * Writes to SMPLRT_DIV (0x19). The formula is rate = 1000 / (1 + value).
 * So writing 0 gives 1kHz, writing 9 gives 100Hz, writing 99 gives 10Hz.
 * Faster rate means more data but more I2C traffic.
 */
DriverStatus MPU6050_Interface::SetSampleRate(const int sample_rate)
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    const uint8_t SMPLRT_DIV = 0x19;
    if (!i2c_->WriteField(addr_, SMPLRT_DIV, 0, 7, static_cast<uint8_t>(sample_rate)))
        return DriverStatus::ERR_I2C_WRITE;

    return calc_sample_rate(sample_rate);
}

/**
 *
 * Reset() — Full device reset. Sets bit 7 of PWR_MGMT_1 (0x6B) to 1.
 * This resets all registers back to their power-on defaults.
 * You'd call Init() again after this. Useful if the chip gets into a weird state.
 */
DriverStatus MPU6050_Interface::Reset()
{
    if (!i2c_)
    {
        return DriverStatus::ERR_NULL_BUS;
    }
}
