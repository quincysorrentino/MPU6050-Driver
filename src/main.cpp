#include "headers/LinuxI2CBus.h"
#include "headers/MPU6050.h"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>

static float wrap_angle_pi(float a)
{
  while (a > M_PI)
    a -= 2.0f * M_PI;
  while (a < -M_PI)
    a += 2.0f * M_PI;
  return a;
}

static void print_table(const IMU_Data &d, float roll, float pitch, float yaw, int sample)
{
  std::cout << "\033[H\033[J";
  std::cout << "MPU6050 Live Readings  (sample " << sample << ")\n";
  std::cout << "+-----------+----------+----------+----------+\n";
  std::cout << "| Sensor    |    X     |    Y     |    Z     |\n";
  std::cout << "+-----------+----------+----------+----------+\n";

  auto cell = [](float v)
  {
    std::cout << " " << std::setw(7) << std::fixed << std::setprecision(3) << v << "  |";
  };

  std::cout << "| Accel (g) |";
  cell(d.ax);
  cell(d.ay);
  cell(d.az);
  std::cout << "\n";
  std::cout << "| Gyro(d/s) |";
  cell(d.gx);
  cell(d.gy);
  cell(d.gz);
  std::cout << "\n";
  std::cout << "+-----------+----------+----------+----------+\n";
  std::cout << "| Temp (C)  | " << std::setw(7) << std::fixed << std::setprecision(2)
            << d.temp << "                            |\n";
  std::cout << "+-----------+----------+----------+----------+\n";
  std::cout << "\n";
  std::cout << "Roll:  " << std::setw(7) << std::fixed << std::setprecision(1) << roll * (180.0f / M_PI) << " deg\n";
  std::cout << "Pitch: " << std::setw(7) << std::fixed << std::setprecision(1) << pitch * (180.0f / M_PI) << " deg\n";
  std::cout << "Yaw:   " << std::setw(7) << std::fixed << std::setprecision(1) << yaw * (180.0f / M_PI) << " deg\n";
  std::cout << "Press Ctrl+C to exit.\n";
  std::cout.flush();
}

int main()
{
  LinuxI2CBus i2c("/dev/i2c-1");
  MPU6050_Interface mpu(&i2c);

  if (mpu.Initialize() != DriverStatus::OK)
  {
    std::cerr << "Init failed\n";
    return 1;
  }

  mpu.Reset();
  mpu.SetAccelRange(0);
  mpu.SetGyroRange(0);
  mpu.SetDLPF(3);

  std::cout << "Keep sensor flat and still — calibrating for 5 seconds...\n";
  std::this_thread::sleep_for(std::chrono::seconds(5));
  if (mpu.Calibrate() != DriverStatus::OK)
  {
    std::cerr << "Calibration failed\n";
    return 1;
  }
  std::cout << "Calibration done.\n";

  float gy_bias = 0.0f;
  float gz_bias = 0.0f;
  float ax_ref = 0.0f;
  float ay_ref = 0.0f;
  float az_ref = 0.0f;
  {
    int valid = 0;
    for (int i = 0; i < 100; i++)
    {
      IMU_Data s{};
      if (mpu.Read(&s) == DriverStatus::OK)
      {
        ax_ref += s.ax;
        ay_ref += s.ay;
        az_ref += s.az;
        gy_bias += s.gy;
        gz_bias += s.gz;
        valid++;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (valid > 0)
    {
      ax_ref /= valid;
      ay_ref /= valid;
      az_ref /= valid;
      gy_bias /= valid;
      gz_bias /= valid;
    }
  }

  const float roll_zero = atan2f(ay_ref, az_ref);
  const float pitch_zero = atan2f(-ax_ref, sqrtf(ay_ref * ay_ref + az_ref * az_ref));
  const float yaw_zero = 0.0f;

  float yaw = 0.0f;
  auto last_t = std::chrono::steady_clock::now();
  const float DEG2RAD = M_PI / 180.0f;
  const float GYRO_DEADZONE = 0.7f;
  int sample = 0;

  while (true)
  {
    IMU_Data d{};
    if (mpu.Read(&d) != DriverStatus::OK)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_t).count();
    last_t = now;

    if (dt <= 0.0f || dt > 0.5f)
    {
      dt = 0.1f;
    }

    float roll = atan2f(d.ay, d.az);
    float pitch = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az));

    float gy_corrected = d.gy - gy_bias;
    float gz_corrected = d.gz - gz_bias;

    float gy = fabsf(gy_corrected) > GYRO_DEADZONE ? gy_corrected : 0.0f;
    float gz = fabsf(gz_corrected) > GYRO_DEADZONE ? gz_corrected : 0.0f;

    float q = gy * DEG2RAD;
    float r = gz * DEG2RAD;
    float cos_pitch = cosf(pitch);
    if (fabsf(cos_pitch) > 0.05f)
    {
      float yaw_rate = (sinf(roll) * q + cosf(roll) * r) / cos_pitch;
      yaw += yaw_rate * dt;
    }

    float roll_rel = wrap_angle_pi(roll - roll_zero);
    float pitch_rel = wrap_angle_pi(pitch - pitch_zero);
    float yaw_rel = wrap_angle_pi(yaw - yaw_zero);

    print_table(d, roll_rel, pitch_rel, yaw_rel, ++sample);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
