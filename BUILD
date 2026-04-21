load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

cc_library(
    name = "i2c_bus",
    hdrs = [
        "src/common/I2CBus.h",
        "src/common/status.h",
    ],
    strip_include_prefix = "src",
)

cc_library(
    name = "linux_i2c_bus",
    srcs = ["src/common/LinuxI2CBus.cpp"],
    hdrs = ["src/common/LinuxI2CBus.h"],
    strip_include_prefix = "src",
    deps = [":i2c_bus"],
)

cc_library(
    name = "test_i2c_bus",
    srcs = ["src/common/TestI2CBus.cpp"],
    hdrs = ["src/common/TestI2CBus.h"],
    strip_include_prefix = "src",
    deps = [":i2c_bus"],
)

cc_library(
    name = "mpu6050",
    srcs = ["src/mpu6050/MPU6050.cpp"],
    hdrs = ["src/mpu6050/MPU6050.h"],
    strip_include_prefix = "src",
    deps = [":i2c_bus"],
)

cc_library(
    name = "bme280",
    hdrs = ["src/bme280/BME280.h"],
    strip_include_prefix = "src",
    deps = [":linux_i2c_bus"],
)

cc_binary(
    name = "mpu_driver",
    srcs = ["src/main.cpp"],
    deps = [":mpu6050", ":linux_i2c_bus"],
)

cc_test(
    name = "mpu6050_test",
    srcs = ["tests/mpu6050/mpu6050_test.cpp"],
    deps = [
        ":mpu6050",
        ":test_i2c_bus",
        "@googletest//:gtest_main",
    ],
)
