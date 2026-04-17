#include "headers/I2C.h"
#include "headers/MPU6050.h"

#include <iostream>
#include <chrono>
#include <thread>

int main() {
  LinuxI2c i2c("/dev/i2c-1");
  MPU6050_Interface mpu(&i2c);

  if (mpu.Initialize() != DriverStatus::OK) {
    std::cerr << "Init failed\n";
    return 1;
  }
  
  return 0;
}