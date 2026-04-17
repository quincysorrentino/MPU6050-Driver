load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "i2c",
    srcs = ["src/I2C.cpp"],
    hdrs = ["src/headers/I2C.h"],
    strip_include_prefix = "src",
)

cc_library(
    name = "mpu6050",
    srcs = ["src/MPU6050.cpp"],
    hdrs = ["src/headers/MPU6050.h"],
    strip_include_prefix = "src",
    deps = [":i2c"],
)

cc_binary(
    name = "mpu_driver",
    srcs = ["src/main.cpp"],
    deps = [":mpu6050"],
)
