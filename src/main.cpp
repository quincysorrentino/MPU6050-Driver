#include "common/LinuxI2CBus.h"
#include "mpu6050/MPU6050.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

int main()
{
    LinuxI2CBus i2c("/dev/i2c-1");
    MPU6050_Interface mpu(&i2c);

    if (mpu.Initialize() != DriverStatus::OK) {
        std::cerr << "Init failed\n";
        return 1;
    }

    mpu.SetAccelRange(0);
    mpu.SetGyroRange(0);
    mpu.SetDLPF(3);

    std::cout << "Calibrating...\n";
    if (mpu.Calibrate() != DriverStatus::OK) {
        std::cerr << "Calibration failed\n";
        return 1;
    }
    std::cout << "Calibration Complete\n\n";

    while (true) {
        IMU_Data d{};
        if (mpu.Read(&d) != DriverStatus::OK) {
            std::cerr << "Read failed\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        std::cout << std::fixed << std::setprecision(3)
                  << "Accel(g)  x=" << std::setw(7) << d.ax
                  << "  y=" << std::setw(7) << d.ay
                  << "  z=" << std::setw(7) << d.az
                  << "   Gyro(d/s)  x=" << std::setw(7) << d.gx
                  << "  y=" << std::setw(7) << d.gy
                  << "  z=" << std::setw(7) << d.gz
                  << "   Temp=" << std::setprecision(1) << d.temp << "C\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
