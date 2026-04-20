#include <gtest/gtest.h>

#include "../src/headers/TestI2CBus.h"
#include "../src/headers/MPU6050.h"

// Helper: returns a TestI2CBus pre-seeded so Initialize() succeeds
// WHO_AM_I register (0x75) must return 0x68
static TestI2CBus MakeHealthyBus() {
    TestI2CBus bus;
    bus.SetRegister(0x75, 0x68);
    return bus;
}

// ───── Initialize ─────

TEST(MPU6050, Initialize_NullBus_ReturnsErrNullBus) {
    MPU6050_Interface driver(nullptr);
    EXPECT_EQ(driver.Initialize(), DriverStatus::ERR_NULL_BUS);
}

TEST(MPU6050, Initialize_HealthyBus_ReturnsOK) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    EXPECT_EQ(driver.Initialize(), DriverStatus::OK);
}

TEST(MPU6050, Initialize_WrongWhoAmI_Fails) {
    TestI2CBus bus;
    bus.SetRegister(0x75, 0xAB); // wrong device ID
    MPU6050_Interface driver(&bus);
    EXPECT_NE(driver.Initialize(), DriverStatus::OK);
}

TEST(MPU6050, Initialize_WakeFails_ReturnsErrI2CWrite) {
    TestI2CBus bus = MakeHealthyBus();
    bus.SetRegisterBehavior(0x6B, RegBehavior::FAIL_WRITE); // PWR_MGMT_1
    MPU6050_Interface driver(&bus);
    EXPECT_EQ(driver.Initialize(), DriverStatus::ERR_I2C_WRITE);
}

TEST(MPU6050, Initialize_WhoAmIReadFails_ReturnsErrI2CRead) {
    TestI2CBus bus = MakeHealthyBus();
    bus.SetRegisterBehavior(0x75, RegBehavior::FAIL_READ);
    MPU6050_Interface driver(&bus);
    EXPECT_EQ(driver.Initialize(), DriverStatus::ERR_I2C_READ);
}

// ───── Not-initialized guard ─────

TEST(MPU6050, Read_BeforeInit_ReturnsErrNotInit) {
    TestI2CBus bus;
    MPU6050_Interface driver(&bus);
    IMU_Data data{};
    EXPECT_EQ(driver.Read(&data), DriverStatus::ERR_NOT_INIT);
}

TEST(MPU6050, SetAccelRange_BeforeInit_ReturnsErrNotInit) {
    TestI2CBus bus;
    MPU6050_Interface driver(&bus);
    EXPECT_EQ(driver.SetAccelRange(0), DriverStatus::ERR_NOT_INIT);
}

TEST(MPU6050, SetGyroRange_BeforeInit_ReturnsErrNotInit) {
    TestI2CBus bus;
    MPU6050_Interface driver(&bus);
    EXPECT_EQ(driver.SetGyroRange(0), DriverStatus::ERR_NOT_INIT);
}

// ───── SetAccelRange ─────

TEST(MPU6050, SetAccelRange_BadParam_ReturnsErrBadParam) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    EXPECT_EQ(driver.SetAccelRange(-1), DriverStatus::ERR_BAD_PARAM);
    EXPECT_EQ(driver.SetAccelRange(4),  DriverStatus::ERR_BAD_PARAM);
}

TEST(MPU6050, SetAccelRange_WriteFails_ReturnsErrI2CWrite) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x1C, RegBehavior::FAIL_WRITE); // ACCEL_CONFIG
    EXPECT_EQ(driver.SetAccelRange(1), DriverStatus::ERR_I2C_WRITE);
}

TEST(MPU6050, SetAccelRange_ReadbackFails_ReturnsErrI2CRead) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x1C, RegBehavior::FAIL_READ);
    EXPECT_EQ(driver.SetAccelRange(1), DriverStatus::ERR_I2C_READ);
}

TEST(MPU6050, SetAccelRange_ValidSettings_ReturnOK) {
    for (int setting = 0; setting <= 3; setting++) {
        TestI2CBus bus = MakeHealthyBus();
        MPU6050_Interface driver(&bus);
        driver.Initialize();
        EXPECT_EQ(driver.SetAccelRange(setting), DriverStatus::OK);
    }
}

// ───── SetGyroRange ─────

TEST(MPU6050, SetGyroRange_BadParam_ReturnsErrBadParam) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    EXPECT_EQ(driver.SetGyroRange(-1), DriverStatus::ERR_BAD_PARAM);
    EXPECT_EQ(driver.SetGyroRange(4),  DriverStatus::ERR_BAD_PARAM);
}

TEST(MPU6050, SetGyroRange_WriteFails_ReturnsErrI2CWrite) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x1B, RegBehavior::FAIL_WRITE); // GYRO_CONFIG
    EXPECT_EQ(driver.SetGyroRange(1), DriverStatus::ERR_I2C_WRITE);
}

TEST(MPU6050, SetGyroRange_ValidSettings_ReturnOK) {
    for (int setting = 0; setting <= 3; setting++) {
        TestI2CBus bus = MakeHealthyBus();
        MPU6050_Interface driver(&bus);
        driver.Initialize();
        EXPECT_EQ(driver.SetGyroRange(setting), DriverStatus::OK);
    }
}

// ───── SetDLPF ─────

TEST(MPU6050, SetDLPF_BadParam_ReturnsErrBadParam) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    EXPECT_EQ(driver.SetDLPF(-1), DriverStatus::ERR_BAD_PARAM);
    EXPECT_EQ(driver.SetDLPF(7),  DriverStatus::ERR_BAD_PARAM);
}

TEST(MPU6050, SetDLPF_ValidSettings_ReturnOK) {
    for (int cfg = 0; cfg <= 6; cfg++) {
        TestI2CBus bus = MakeHealthyBus();
        MPU6050_Interface driver(&bus);
        driver.Initialize();
        EXPECT_EQ(driver.SetDLPF(cfg), DriverStatus::OK);
    }
}

// ───── Read ─────

TEST(MPU6050, Read_SensorReadFails_ReturnsErrI2CRead) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x3B, RegBehavior::FAIL_READ); // ACCEL_XOUT_H
    IMU_Data data{};
    EXPECT_EQ(driver.Read(&data), DriverStatus::ERR_I2C_READ);
}

TEST(MPU6050, Read_KnownRawAccel_ScalesCorrectly) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();

    // ax raw = 0x4000 = 16384. At default ±2g scale (16384 LSB/g) → 1.0g
    bus.SetRegister(0x3B, 0x40); // ax high byte
    bus.SetRegister(0x3C, 0x00); // ax low byte
    // remaining registers default to 0

    IMU_Data data{};
    ASSERT_EQ(driver.Read(&data), DriverStatus::OK);
    EXPECT_NEAR(data.ax, 1.0f, 0.001f);
}

// ───── Sleep / Wake ─────

TEST(MPU6050, Sleep_BeforeInit_ReturnsErrNotInit) {
    TestI2CBus bus;
    MPU6050_Interface driver(&bus);
    EXPECT_EQ(driver.Sleep(), DriverStatus::ERR_NOT_INIT);
}

TEST(MPU6050, Sleep_WriteFails_ReturnsErrI2CWrite) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x6B, RegBehavior::FAIL_WRITE);
    EXPECT_EQ(driver.Sleep(), DriverStatus::ERR_I2C_WRITE);
}

// ───── Calibrate ─────

TEST(MPU6050, Calibrate_BeforeInit_ReturnsErrNotInit) {
    TestI2CBus bus;
    MPU6050_Interface driver(&bus);
    EXPECT_EQ(driver.Calibrate(), DriverStatus::ERR_NOT_INIT);
}

TEST(MPU6050, Calibrate_SensorReadFails_ReturnsErrI2CRead) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x3B, RegBehavior::FAIL_READ); // ACCEL_XOUT_H
    EXPECT_EQ(driver.Calibrate(), DriverStatus::ERR_I2C_READ);
}

TEST(MPU6050, Calibrate_GyroOffsetWriteFails_ReturnsErrI2CWrite) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x13, RegBehavior::FAIL_WRITE); // XG_OFFS_USRH
    EXPECT_EQ(driver.Calibrate(), DriverStatus::ERR_I2C_WRITE);
}

TEST(MPU6050, Calibrate_AccelTrimReadFails_ReturnsErrI2CRead) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x07, RegBehavior::FAIL_READ); // XA_OFFS_L
    EXPECT_EQ(driver.Calibrate(), DriverStatus::ERR_I2C_READ);
}

TEST(MPU6050, Calibrate_AccelOffsetWriteFails_ReturnsErrI2CWrite) {
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    bus.SetRegisterBehavior(0x06, RegBehavior::FAIL_WRITE); // XA_OFFS_H
    EXPECT_EQ(driver.Calibrate(), DriverStatus::ERR_I2C_WRITE);
}

TEST(MPU6050, Calibrate_ZeroData_ReturnsOK) {
    // all sensor registers default to 0 in TestI2CBus, so offsets will be 0
    TestI2CBus bus = MakeHealthyBus();
    MPU6050_Interface driver(&bus);
    driver.Initialize();
    EXPECT_EQ(driver.Calibrate(), DriverStatus::OK);
}
