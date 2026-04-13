#include "headers/I2C.h"
#include "headers/MPU6050.h"

#include <iostream>
#include <chrono>
#include <thread>

int main() {
  LinuxI2c i2c("/dev/i2c-1");
  MPU6050_Interface mpu(&i2c);

  if (!mpu.Initialize()) {
    std::cerr << "Init failed\n";
    return 1;
  }

  int16_t gx = 0, gy = 0, gz = 0;

  while(true){
    if (mpu.ReadGyro(&gx, &gy, &gz)) {
      std::cout << "Gyro: " << gx << ", " << gy << ", " << gz << "\n";
    } else {
      std::cerr << "Read failed\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  
  return 0;
}