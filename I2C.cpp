#include "headers/I2C.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <iostream>

LinuxI2c::LinuxI2c(const std::string& device_path) {
  fd_ = open(device_path.c_str(), O_RDWR);
  if (fd_ < 0) {
    std::cerr << "Failed to open I2C device: " << device_path << "\n";
  }
}

LinuxI2c::~LinuxI2c() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool LinuxI2c::SetSlaveAddr(uint8_t addr) {
  if (ioctl(fd_, I2C_SLAVE, addr) < 0) {
    return false;
  }
  return true;
}

bool LinuxI2c::WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t value) {
  if (!SetSlaveAddr(dev_addr)) {
    return false;
  }

  uint8_t buf[2] = {reg, value};
  if (write(fd_, buf, 2) != 2) {
    return false;
  }

  return true;
}

bool LinuxI2c::ReadByte(uint8_t dev_addr, uint8_t reg, uint8_t* out) {
  if (!SetSlaveAddr(dev_addr)) {
    return false;
  }

  if (write(fd_, &reg, 1) != 1) {
    return false;
  }

  if (read(fd_, out, 1) != 1) {
    return false;
  }

  return true;
}

bool LinuxI2c::ReadBlock(uint8_t dev_addr, uint8_t reg, size_t count,
                         std::vector<uint8_t>* out) {
  if (!SetSlaveAddr(dev_addr)) {
    return false;
  }

  if (write(fd_, &reg, 1) != 1) {
    return false;
  }

  out->resize(count);
  if (read(fd_, out->data(), count) != static_cast<ssize_t>(count)) {
    return false;
  }

  return true;
}
