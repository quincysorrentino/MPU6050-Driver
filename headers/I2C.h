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

	bool WriteBit(uint8_t dev_addr, uint8_t reg, uint8_t bit_num,
						 uint8_t bit_value);
	bool ReadBlock(uint8_t dev_addr, uint8_t reg, size_t count,
								 std::vector<uint8_t>* out);

 private:
	bool SetSlaveAddr(uint8_t addr);
	bool WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t value);
	bool ReadByte(uint8_t dev_addr, uint8_t reg, uint8_t* out);

	int fd_ = -1;
};
