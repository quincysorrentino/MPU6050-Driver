#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common/I2CBus.h"

enum class RegBehavior {
    OK,
    FAIL_WRITE,
    FAIL_READ,
};

class TestI2CBus : public I2CBus {
 public:
    TestI2CBus() = default;

    // Pre-load a register value before a test
    void SetRegister(uint8_t reg, uint8_t value) { registers_[reg] = value; }
    uint8_t GetRegister(uint8_t reg) const { return registers_[reg]; }

    // Make reads or writes to a specific register fail
    void SetRegisterBehavior(uint8_t reg, RegBehavior behavior) {
        behavior_[reg] = behavior;
    }

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
    uint8_t registers_[256] = {};
    RegBehavior behavior_[256] = {};
};
