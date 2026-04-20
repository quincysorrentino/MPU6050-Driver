load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

cc_library(
    name = "i2c_bus",
    hdrs = ["src/headers/I2CBus.h"],
    strip_include_prefix = "src",
)

cc_library(
    name = "linux_i2c_bus",
    srcs = ["src/LinuxI2CBus.cpp"],
    hdrs = ["src/headers/LinuxI2CBus.h"],
    strip_include_prefix = "src",
    deps = [":i2c_bus"],
)

cc_library(
    name = "mpu6050",
    srcs = ["src/MPU6050.cpp"],
    hdrs = ["src/headers/MPU6050.h", "src/headers/status.h"],
    strip_include_prefix = "src",
    deps = [":i2c_bus"],
)

cc_binary(
    name = "mpu_driver",
    srcs = ["src/main.cpp"],
    deps = [":mpu6050", ":linux_i2c_bus"],
)

cc_library(
    name = "test_i2c_bus",
    srcs = ["src/TestI2CBus.cpp"],
    hdrs = ["src/headers/TestI2CBus.h"],
    strip_include_prefix = "src",
    deps = [":i2c_bus"],
)

cc_test(
    name = "mpu6050_test",
    srcs = ["tests/mpu6050_test.cpp"],
    deps = [
        ":mpu6050",
        ":test_i2c_bus",
        "@googletest//:gtest_main",
    ],
)
