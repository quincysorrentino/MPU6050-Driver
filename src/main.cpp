#include "common/LinuxI2CBus.h"
#include "bme280/BME280.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <string>

int main()
{
    LinuxI2CBus i2c("/dev/i2c-1");
    BME280_Interface bme(&i2c);

    if (bme.Initialize() != DriverStatus::OK) {
        std::cerr << "Init failed\n";
        return 1;
    }

    std::cout << "BME280 ready\n\n";

    while (true) {
        BME280Data d{};
        if (bme.Read(&d) != DriverStatus::OK) {
            std::cerr << "Read failed\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        double temp_c    = d.temperature / 100.0;
        double press_hpa = (d.pressure / 256.0) / 100.0;
        double hum_pct   = d.humidity / 1024.0;

        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

        std::cout << "{"
          << "\"ts\":"       << timestamp   << ","
          << "\"temp\":"     << temp_c      << ","
          << "\"pressure\":" << press_hpa   << ","
          << "\"humidity\":" << hum_pct
          << "}\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
