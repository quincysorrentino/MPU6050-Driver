#include "TestI2CBus.h"

bool TestI2CBus::WriteBit(uint8_t, uint8_t reg,
                          uint8_t bit_num, uint8_t bit_value)
{
    if (behavior_[reg] == RegBehavior::FAIL_WRITE) return false;
    if (bit_value) registers_[reg] |=  (1u << bit_num);
    else           registers_[reg] &= ~(1u << bit_num);
    return true;
}

bool TestI2CBus::WriteField(uint8_t, uint8_t reg,
                            uint8_t bit_start, uint8_t bit_width,
                            uint8_t value)
{
    if (behavior_[reg] == RegBehavior::FAIL_WRITE) return false;
    uint8_t mask = static_cast<uint8_t>(((1u << bit_width) - 1u) << bit_start);
    registers_[reg] = static_cast<uint8_t>(
        (registers_[reg] & ~mask) | ((value << bit_start) & mask));
    return true;
}

bool TestI2CBus::ReadField(uint8_t, uint8_t reg,
                           uint8_t bit_start, uint8_t bit_width,
                           uint8_t* out)
{
    if (behavior_[reg] == RegBehavior::FAIL_READ) return false;
    uint8_t mask = static_cast<uint8_t>((1u << bit_width) - 1u);
    *out = (registers_[reg] >> bit_start) & mask;
    return true;
}

bool TestI2CBus::ReadBlock(uint8_t, uint8_t reg,
                           size_t count, std::vector<uint8_t>* out)
{
    for (size_t i = 0; i < count; i++) {
        if (behavior_[reg + i] == RegBehavior::FAIL_READ) return false;
    }
    out->clear();
    for (size_t i = 0; i < count; i++)
        out->push_back(registers_[reg + i]);
    return true;
}
