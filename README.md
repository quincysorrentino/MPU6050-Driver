# MPU6050-Driver

PWR_MGMT_1  0x6B    Write 0x00 to wake from sleep(resets to 0x40 = sleep mode)
WHO_AM_I    0x75    Read to verify device (returns 0x68)
ACCEL_XOUT_H    0x3B    Start of 14-byte burst read (accel + temp + gyro)
GYRO_XOUT_H    0x43    Gyro data start (you get this in the burst read anyway)
TEMP_OUT_H     0x41     Temp data (also in the burst read)
ACCEL_CONFIG    0x1C    Set accel range (bits 4:3 = AFS_SEL)
GYRO_CONFIG     0x1B    Set gyro range (bits 4:3 = FS_SEL)