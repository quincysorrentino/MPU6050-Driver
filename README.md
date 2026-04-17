# MPU6050-Driver

Initialize() — Wake the chip and verify identity. Writes 0x00 to PWR_MGMT_1 (0x6B) to clear the SLEEP bit, then reads WHO_AM_I (0x75) to confirm you're talking to a real MPU6050. You should call this once before anything else.

Read() — Get all sensor data in one shot. Reads 14 bytes starting at ACCEL_XOUT_H (0x3B). Those 14 bytes are packed in order: accel X/Y/Z (6 bytes), temperature (2 bytes), gyro X/Y/Z (6 bytes). Each value is two bytes, big-endian, signed. You combine them, then divide by the scale factor to get real units.

SetAccelRange() — Change accelerometer sensitivity. Writes to ACCEL_CONFIG (0x1C), specifically bits 4:3. The options are ±2g, ±4g, ±8g, ±16g. A wider range means you can measure bigger forces but with less precision. At ±2g you divide raw values by 16384 to get g's. At ±16g you divide by 2048.

SetGyroRange() — Same idea for the gyroscope. Writes to GYRO_CONFIG (0x1B), bits 4:3. Options are ±250, ±500, ±1000, ±2000 degrees per second. A slow-spinning thing like a phone needs ±250. A fast-spinning drone motor needs ±2000.

SetSampleRate() — Controls how often the chip takes a new measurement. Writes to SMPLRT_DIV (0x19). The formula is rate = 1000 / (1 + value). So writing 0 gives 1kHz, writing 9 gives 100Hz, writing 99 gives 10Hz. Faster rate means more data but more I2C traffic.

Reset() — Full device reset. Sets bit 7 of PWR_MGMT_1 (0x6B) to 1. This resets all registers back to their power-on defaults. You'd call Init() again after this. Useful if the chip gets into a weird state.

Sleep() / Wake() — Put the chip to sleep or wake it up. Both write to PWR_MGMT_1 (0x6B), setting or clearing bit 6. Sleep mode cuts power consumption dramatically when you don't need readings.

IsDataReady() — Check if new data is available. Reads INT_STATUS (0x3A) and checks bit 0. Returns true when the chip has finished a new measurement. Useful if you don't want to read stale data.