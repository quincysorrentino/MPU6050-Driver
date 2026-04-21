#include "BME280.h"
#include <chrono>

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
        if (i2c_->ReadField(addr_, id_register, 7, 7, &id_val))
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
        if (i2c_->WriteField(addr_, reset_register, 7, 7, 0xB6))
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
