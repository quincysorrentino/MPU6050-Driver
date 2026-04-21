#pragma once

#include <cstdint>

enum class DriverStatus : uint8_t {
    OK            = 0,
    ERR_NULL_BUS  = 1,
    ERR_I2C_READ  = 2,
    ERR_I2C_WRITE = 3,
    ERR_BAD_PARAM = 4,
    ERR_BAD_DATA = 5,
    ERR_VERIFY_FAILED = 6,
    ERR_NOT_INIT = 7,
};