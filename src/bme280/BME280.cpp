#include "BME280.h"
#include <chrono>
#include <vector>

BME280_Interface::BME280_Interface(I2CBus *i2c, uint8_t address)
    : i2c_(i2c), addr_(address) {}

DriverStatus BME280_Interface::Initialize()
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    uint8_t chip_id = 0;
    DriverStatus id_status = GET_ID(&chip_id);
    if (id_status != DriverStatus::OK)
        return id_status;

    if (chip_id != 0x60)
        return DriverStatus::ERR_BAD_DATA;

    DriverStatus cal_status = LoadCalibration();
    if (cal_status != DriverStatus::OK)
        return cal_status;

    // ctrl_hum (0xF2): humidity oversampling ×1 — must be written before ctrl_meas
    bool ok = false;
    for (int retry = 0; retry < 4; retry++) {
        if (i2c_->WriteField(addr_, 0xF2, 0, 8, 0x01)) { ok = true; break; }
    }
    if (!ok) return DriverStatus::ERR_I2C_WRITE;

    // ctrl_meas (0xF4): temp ×1 | pressure ×1 | normal mode
    // osrs_t[7:5]=001, osrs_p[4:2]=001, mode[1:0]=11 → 0x27
    ok = false;
    for (int retry = 0; retry < 4; retry++) {
        if (i2c_->WriteField(addr_, 0xF4, 0, 8, 0x27)) { ok = true; break; }
    }
    if (!ok) return DriverStatus::ERR_I2C_WRITE;

    initialized_ = true;

    return DriverStatus::OK;
}

DriverStatus BME280_Interface::GET_ID(uint8_t *out)
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    const uint8_t id_register = 0xD0;

    uint8_t id_val = 0;
    bool valid_read = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadField(addr_, id_register, 0, 8, &id_val))
        {
            valid_read = true;
            break;
        }
    }

    if (!valid_read)
        return DriverStatus::ERR_I2C_READ;

    *out = id_val;
    return DriverStatus::OK;
}

DriverStatus BME280_Interface::RESET()
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    if (!initialized_)
        return DriverStatus::ERR_NOT_INIT;

    const uint8_t reset_register = 0xE0;

    bool valid_write = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->WriteField(addr_, reset_register, 0, 8, 0xB6))
        {
            valid_write = true;
            break;
        }
    }

    if (!valid_write)
        return DriverStatus::ERR_I2C_WRITE;

    initialized_ = false;

    auto start_delay = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_delay < std::chrono::milliseconds(100))
    {
        // wait
    }

    return Initialize();
}

DriverStatus BME280_Interface::Read(BME280Data* out)
{
    if (!i2c_)
        return DriverStatus::ERR_NULL_BUS;

    if (!initialized_)
        return DriverStatus::ERR_NOT_INIT;

    if (!out)
        return DriverStatus::ERR_BAD_PARAM;

    std::vector<uint8_t> buf;
    bool valid_read = false;
    for (int retry = 0; retry < 4; retry++)
    {
        if (i2c_->ReadBlock(addr_, 0xF7, 8, &buf))
        {
            valid_read = true;
            break;
        }
    }

    if (!valid_read)
        return DriverStatus::ERR_I2C_READ;

    out->raw_pressure = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4);
    out->raw_temperature = ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4);
    out->raw_humidity = ((uint16_t)buf[6] << 8)  |  buf[7];

    int32_t t_fine = 0;
    out->temperature = CompensateTemperature((int32_t)out->raw_temperature, t_fine);
    out->pressure = CompensatePressure((int32_t)out->raw_pressure, t_fine);
    out->humidity = CompensateHumidity((int32_t)out->raw_humidity, t_fine);

    return DriverStatus::OK;
}

DriverStatus BME280_Interface::LoadCalibration()
{
    std::vector<uint8_t> tp;
    bool ok = false;
    for (int retry = 0; retry < 4; retry++) {
        if (i2c_->ReadBlock(addr_, 0x88, 24, &tp)) { ok = true; break; }
    }
    if (!ok) return DriverStatus::ERR_I2C_READ;

    calib_.dig_T1 = (uint16_t)((tp[1] << 8) | tp[0]);
    calib_.dig_T2 = (int16_t)((tp[3] << 8) | tp[2]);
    calib_.dig_T3 = (int16_t)((tp[5] << 8) | tp[4]);
    calib_.dig_P1 = (uint16_t)((tp[7] << 8) | tp[6]);
    calib_.dig_P2 = (int16_t)((tp[9]  << 8) | tp[8]);
    calib_.dig_P3 = (int16_t)((tp[11] << 8) | tp[10]);
    calib_.dig_P4 = (int16_t)((tp[13] << 8) | tp[12]);
    calib_.dig_P5 = (int16_t)((tp[15] << 8) | tp[14]);
    calib_.dig_P6 = (int16_t)((tp[17] << 8) | tp[16]);
    calib_.dig_P7 = (int16_t)((tp[19] << 8) | tp[18]);
    calib_.dig_P8 = (int16_t)((tp[21] << 8) | tp[20]);
    calib_.dig_P9 = (int16_t)((tp[23] << 8) | tp[22]);

    std::vector<uint8_t> h1;
    ok = false;
    for (int retry = 0; retry < 4; retry++) {
        if (i2c_->ReadBlock(addr_, 0xA1, 1, &h1)) { ok = true; break; }
    }
    if (!ok) return DriverStatus::ERR_I2C_READ;
    calib_.dig_H1 = h1[0];

    std::vector<uint8_t> h;
    ok = false;
    for (int retry = 0; retry < 4; retry++) {
        if (i2c_->ReadBlock(addr_, 0xE1, 7, &h)) { ok = true; break; }
    }
    if (!ok) return DriverStatus::ERR_I2C_READ;

    calib_.dig_H2 = (int16_t)((h[1] << 8) | h[0]);
    calib_.dig_H3 = h[2];
    calib_.dig_H4 = (int16_t)((h[3] << 4) | (h[4] & 0x0F));
    calib_.dig_H5 = (int16_t)((h[5] << 4) | (h[4] >> 4));
    calib_.dig_H6 = (int8_t)h[6];

    return DriverStatus::OK;
}

int32_t BME280_Interface::CompensateTemperature(int32_t adc_T, int32_t& t_fine)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib_.dig_T1 << 1))) *
                    (int32_t)calib_.dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - (int32_t)calib_.dig_T1) *
                      ((adc_T >> 4) - (int32_t)calib_.dig_T1)) >> 12) *
                    (int32_t)calib_.dig_T3) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

uint32_t BME280_Interface::CompensatePressure(int32_t adc_P, int32_t t_fine)
{
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib_.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_.dig_P5) << 17);
    var2 = var2 + ((int64_t)calib_.dig_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)calib_.dig_P3) >> 8) +
           ((var1 * (int64_t)calib_.dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * (int64_t)calib_.dig_P1 >> 33;
    if (var1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)calib_.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)calib_.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)calib_.dig_P7 << 4);
    return (uint32_t)p;
}

uint32_t BME280_Interface::CompensateHumidity(int32_t adc_H, int32_t t_fine)
{
    int32_t x = t_fine - 76800;
    x = (((((adc_H << 14) - ((int32_t)calib_.dig_H4 << 20) -
            ((int32_t)calib_.dig_H5 * x)) + 16384) >> 15) *
         (((((((x * (int32_t)calib_.dig_H6) >> 10) *
              (((x * (int32_t)calib_.dig_H3) >> 11) + 32768)) >> 10) +
            2097152) * (int32_t)calib_.dig_H2 + 8192) >> 14));
    x -= (((((x >> 15) * (x >> 15)) >> 7) * (int32_t)calib_.dig_H1) >> 4);
    x = x < 0 ? 0 : x;
    x = x > 419430400 ? 419430400 : x;
    return (uint32_t)(x >> 12);
}
