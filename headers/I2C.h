// I2C interface for Linux I2C-dev.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class LinuxI2c {
 public:
	explicit LinuxI2c(const std::string& device_path);
	~LinuxI2c();

	// Single-bit read-modify-write. bit_num: 0 = LSB, 7 = MSB.
	bool WriteBit(uint8_t dev_addr, uint8_t reg, uint8_t bit_num,
						 uint8_t bit_value);

	// Multi-bit read-modify-write. bit_start is the LSB of the field,
	// bit_width is the number of bits. value is unshifted (LSB-aligned).
	bool WriteField(uint8_t dev_addr, uint8_t reg, uint8_t bit_start,
							uint8_t bit_width, uint8_t value);
	bool ReadField(uint8_t dev_addr, uint8_t reg, uint8_t bit_start,
						  uint8_t bit_width, uint8_t* out);

	bool ReadBlock(uint8_t dev_addr, uint8_t reg, size_t count,
								 std::vector<uint8_t>* out);

 private:
	bool SetSlaveAddr(uint8_t addr);
	bool WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t value);
	bool ReadByte(uint8_t dev_addr, uint8_t reg, uint8_t* out);

	int fd_ = -1;
};
