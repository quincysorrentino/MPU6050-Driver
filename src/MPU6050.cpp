#include "headers/MPU6050.h"

#include <vector>
#include <cstdint>
#include <chrono>
#include <cmath>
#include "headers/I2CBus.h"

MPU6050_Interface::MPU6050_Interface(I2CBus *i2c, uint8_t address)
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

    DriverStatus wake_status = Wake();
    if (wake_status != DriverStatus::OK)
    {
        return wake_status;
    }

    // set clock source
    const uint8_t clock_source_ppl = 0x01;

    bool clock_set = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, PWR_MGMT_1, 0, 3, clock_source_ppl))
        {
            clock_set = true;
            break;
        }
    }

    if (!clock_set)
        return DriverStatus::ERR_I2C_WRITE;

    // check who_am_i
    uint8_t who_am_i_check;
    WHO_AM_I(&who_am_i_check);

    if (who_am_i_check != 0x68)
    {
        return DriverStatus::ERR_I2C_READ;
    }

    initialized_ = true;

    return DriverStatus::OK;
}

DriverStatus MPU6050_Interface::WHO_AM_I(uint8_t *out) const
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    const uint8_t WHO_AM_I_REG = 0x75;
    std::vector<uint8_t> buf;

    bool valid_read = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadBlock(addr_, WHO_AM_I_REG, 1, &buf))
        {
            valid_read = true;
            break;
        }
    }

    if (!valid_read)
    {
        return DriverStatus::ERR_I2C_READ;
    }

    *out = buf[0];
    return DriverStatus::OK;
}

DriverStatus MPU6050_Interface::ReadRaw(IMU_Raw *out)
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    if (!initialized_)
    {
        return DriverStatus::ERR_NOT_INIT;
    }

    const uint8_t ACCEL_XOUT_H = 0x3B;
    std::vector<uint8_t> buf;

    bool valid_read = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadBlock(addr_, ACCEL_XOUT_H, 14, &buf))
        {
            valid_read = true;
            break;
        }
    }

    if (!valid_read)
    {
        return DriverStatus::ERR_I2C_READ;
    }

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

    if (!initialized_)
    {
        return DriverStatus::ERR_NOT_INIT;
    }

    const uint8_t ACCEL_CONFIG = 0x1C;

    bool valid_write = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, ACCEL_CONFIG, 3, 2, static_cast<uint8_t>(setting)))
        {
            valid_write = true;
            break;
        }
    }
    if (!valid_write)
        return DriverStatus::ERR_I2C_WRITE;

    uint8_t verify_val;

    bool valid_read = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadField(addr_, ACCEL_CONFIG, 3, 2, &verify_val))
        {
            valid_read = true;
            break;
        }
    }

    if (!valid_read)
        return DriverStatus::ERR_I2C_READ;

    if (verify_val != static_cast<uint8_t>(setting))
        return DriverStatus::ERR_VERIFY_FAILED;

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

    if (!initialized_)
    {
        return DriverStatus::ERR_NOT_INIT;
    }

    const uint8_t GYRO_CONFIG = 0x1B;

    bool valid_write = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, GYRO_CONFIG, 3, 2, static_cast<uint8_t>(FS_SEL_SETTING)))
        {
            valid_write = true;
            break;
        }
    }
    if (!valid_write)
        return DriverStatus::ERR_I2C_WRITE;

    uint8_t verify_val;

    bool valid_read = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadField(addr_, GYRO_CONFIG, 3, 2, &verify_val))
        {
            valid_read = true;
            break;
        }
    }
    if (!valid_read)
        return DriverStatus::ERR_I2C_READ;

    if (verify_val != static_cast<uint8_t>(FS_SEL_SETTING))
        return DriverStatus::ERR_VERIFY_FAILED;

    static constexpr float kGyroScales[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    gyro_scale_ = kGyroScales[FS_SEL_SETTING];

    return DriverStatus::OK;
}

DriverStatus MPU6050_Interface::calc_sample_rate(const int sample_rate)
{
    const uint8_t CONFIG = 0x1A;
    uint8_t DLPF_CFG_VAL;

    bool valid_read = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadField(addr_, CONFIG, 0, 3, &DLPF_CFG_VAL))
        {
            valid_read = true;
            break;
        }
    }
    if (!valid_read)
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

    if (!initialized_)
    {
        return DriverStatus::ERR_NOT_INIT;
    }

    const uint8_t SMPLRT_DIV = 0x19;

    bool valid_write = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, SMPLRT_DIV, 0, 7, static_cast<uint8_t>(sample_rate)))
        {
            valid_write = true;
            break;
        }
    }
    if (!valid_write)
        return DriverStatus::ERR_I2C_WRITE;

    return calc_sample_rate(sample_rate);
}

/**
 * SetDLPF() — Configure the Digital Low Pass Filter on both gyro and accel.
 * Writes to CONFIG (0x1A) bits 2:0. cfg=0 is no filter (8kHz gyro output rate);
 * cfg=1–6 progressively narrow the bandwidth down to 5Hz at cfg=6 (1kHz output rate).
 * Also recalculates the stored sample rate since DLPF changes the gyro output rate.
 */
DriverStatus MPU6050_Interface::SetDLPF(const int cfg)
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;
    if (!initialized_)
        return DriverStatus::ERR_NOT_INIT;
    if (cfg < 0 || cfg > 6)
        return DriverStatus::ERR_BAD_PARAM;

    const uint8_t CONFIG = 0x1A;
    const uint8_t SMPLRT_DIV = 0x19;

    bool valid_write = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, CONFIG, 0, 3, static_cast<uint8_t>(cfg)))
        {
            valid_write = true;
            break;
        }
    }
    if (!valid_write)
        return DriverStatus::ERR_I2C_WRITE;

    uint8_t verify_val;
    bool valid_read = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadField(addr_, CONFIG, 0, 3, &verify_val))
        {
            valid_read = true;
            break;
        }
    }
    if (!valid_read)
        return DriverStatus::ERR_I2C_READ;

    if (verify_val != static_cast<uint8_t>(cfg))
        return DriverStatus::ERR_VERIFY_FAILED;

    // DLPF change affects gyro output rate, so recalculate the stored sample rate
    uint8_t current_div = 0;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadField(addr_, SMPLRT_DIV, 0, 8, &current_div))
            break;
    }
    return calc_sample_rate(current_div);
}

/**
 *
 * Reset() — Full device reset. Sets bit 7 of PWR_MGMT_1 (0x6B) to 1.
 * This resets all registers back to their power-on defaults.
 * You'd call Initialize() again after this. Useful if the chip gets into a weird state.
 */
DriverStatus MPU6050_Interface::Reset()
{
    if (!i2c_)
    {
        return DriverStatus::ERR_NULL_BUS;
    }

    if (!initialized_)
    {
        return DriverStatus::ERR_NOT_INIT;
    }

    const uint8_t PWR_MGMT_1 = 0x6B;

    bool valid_write = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, PWR_MGMT_1, 7, 1, 1))
        {
            valid_write = true;
            break;
        }
    }
    if (!valid_write)
        return DriverStatus::ERR_I2C_WRITE;

    initialized_ = false;

    // pause for 100ms to let restart process
    auto start_delay = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_delay < std::chrono::milliseconds(100))
    {
        // wait
    }

    DriverStatus init_status = Initialize();

    if (init_status != DriverStatus::OK)
    {
        return init_status;
    }

    return DriverStatus::OK;
}

/**
 * Sleep() / Wake() — Put the chip to sleep or wake it up.
 * Both write to PWR_MGMT_1 (0x6B), setting or clearing bit 6.
 * Sleep mode cuts power consumption dramatically when you don't need readings.
 */

DriverStatus MPU6050_Interface::Sleep()
{
    if (!i2c_)
    {
        return DriverStatus::ERR_NULL_BUS;
    }

    if (!initialized_)
        return DriverStatus::ERR_NOT_INIT;

    const uint8_t PWR_MGMT_1 = 0x6B;

    bool valid_write = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, PWR_MGMT_1, 6, 1, 1))
        {
            valid_write = true;
            break;
        }
    }
    if (!valid_write)
        return DriverStatus::ERR_I2C_WRITE;

    return DriverStatus::OK;
}

DriverStatus MPU6050_Interface::Wake()
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    // No initialized_ guard — Wake() is called by Initialize() before the flag is set

    const uint8_t PWR_MGMT_1 = 0x6B;

    bool valid_write = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, PWR_MGMT_1, 6, 1, 0))
        {
            valid_write = true;
            break;
        }
    }
    if (!valid_write)
        return DriverStatus::ERR_I2C_WRITE;

    return DriverStatus::OK;
}

DriverStatus MPU6050_Interface::Calibrate(){
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;
    

    if (!initialized_)
        return DriverStatus::ERR_NOT_INIT;
    
    // take average of 500 samples
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < 500; i++){
        IMU_Raw raw;

        bool valid_read = false;
        for (int retry = 0; retry < 4; retry++){
            if (ReadRaw(&raw) == DriverStatus::OK){
                valid_read = true;
                break;
            }
        }

        if (!valid_read)
            return DriverStatus::ERR_I2C_READ;

        ax_sum += raw.ax;
        ay_sum += raw.ay;
        az_sum += raw.az;
        gx_sum += raw.gx;
        gy_sum += raw.gy;
        gz_sum += raw.gz;
    }

    // calc offsets (negate mean)
    int16_t ax_off = static_cast<int16_t>(-(ax_sum / 500));
    int16_t ay_off = static_cast<int16_t>(-(ay_sum / 500));
    int16_t az_off = static_cast<int16_t>(-(az_sum / 500));
    int16_t gx_off = static_cast<int16_t>(-(gx_sum / 500));
    int16_t gy_off = static_cast<int16_t>(-(gy_sum / 500));
    int16_t gz_off = static_cast<int16_t>(-(gz_sum / 500));

    // write gyro offsets to 0x13–0x18
    const uint8_t GYRO_OFFSET_BASE = 0x13;
    int16_t gyro_offs[3] = {gx_off, gy_off, gz_off};

    for (int axis = 0; axis < 3; axis++){
        uint8_t high = static_cast<uint8_t>(gyro_offs[axis] >> 8);
        uint8_t low  = static_cast<uint8_t>(gyro_offs[axis] & 0xFF);
        if (!i2c_->WriteField(addr_, GYRO_OFFSET_BASE + axis * 2, 0, 8, high))
            return DriverStatus::ERR_I2C_WRITE;
        if (!i2c_->WriteField(addr_, GYRO_OFFSET_BASE + axis * 2 + 1, 0, 8, low))
            return DriverStatus::ERR_I2C_WRITE;
    }

    // write accel offsets to 0x06–0x0B
    const uint8_t ACCEL_OFFSET_BASE = 0x06;
    int16_t accel_offs[3] = {ax_off, ay_off, az_off};
    for (int axis = 0; axis < 3; axis++){
        uint8_t low_reg = ACCEL_OFFSET_BASE + axis * 2 + 1;

        // read existing low byte to preserve factory trim bit 0
        std::vector<uint8_t> existing;

        if (!i2c_->ReadBlock(addr_, low_reg, 1, &existing))
            return DriverStatus::ERR_I2C_READ;
        uint8_t trim_bit = existing[0] & 0x01;

        int16_t scaled = static_cast<int16_t>(accel_offs[axis] / 8);
        int16_t reg_val = static_cast<int16_t>((scaled << 1) | trim_bit);

        uint8_t high = static_cast<uint8_t>(reg_val >> 8);
        uint8_t low  = static_cast<uint8_t>(reg_val & 0xFF);

        if (!i2c_->WriteField(addr_, ACCEL_OFFSET_BASE + axis * 2, 0, 8, high))
            return DriverStatus::ERR_I2C_WRITE;
        if (!i2c_->WriteField(addr_, low_reg, 0, 8, low))
            return DriverStatus::ERR_I2C_WRITE;
    }

    return DriverStatus::OK;
}