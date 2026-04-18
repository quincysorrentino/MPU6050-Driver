#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

class I2CBus {
 public:
    virtual ~I2CBus() = default;

    virtual bool WriteBit(uint8_t dev_addr, uint8_t reg,
                          uint8_t bit_num, uint8_t bit_value) = 0;

    virtual bool WriteField(uint8_t dev_addr, uint8_t reg,
                            uint8_t bit_start, uint8_t bit_width,
                            uint8_t value) = 0;

    virtual bool ReadField(uint8_t dev_addr, uint8_t reg,
                           uint8_t bit_start, uint8_t bit_width,
                           uint8_t* out) = 0;

    virtual bool ReadBlock(uint8_t dev_addr, uint8_t reg,
                           size_t count, std::vector<uint8_t>* out) = 0;
};
