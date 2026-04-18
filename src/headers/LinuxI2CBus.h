// I2C interface for Linux I2C-dev.
#pragma once

#include "I2CBus.h"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class LinuxI2CBus : public I2CBus {
 public:
    explicit LinuxI2CBus(const std::string& device_path);
    ~LinuxI2CBus() override;

    bool WriteBit(uint8_t dev_addr, uint8_t reg,
                  uint8_t bit_num, uint8_t bit_value) override;
    bool WriteField(uint8_t dev_addr, uint8_t reg,
                    uint8_t bit_start, uint8_t bit_width,
                    uint8_t value) override;
    bool ReadField(uint8_t dev_addr, uint8_t reg,
                   uint8_t bit_start, uint8_t bit_width,
                   uint8_t* out) override;
    bool ReadBlock(uint8_t dev_addr, uint8_t reg,
                   size_t count, std::vector<uint8_t>* out) override;

 private:
    bool SetSlaveAddr(uint8_t addr);
    bool WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t value);
    bool ReadByte(uint8_t dev_addr, uint8_t reg, uint8_t* out);

    int fd_ = -1;
};
