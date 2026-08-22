#ifndef _IMU_MPU6050_H
#define _IMU_MPU6050_H

#include "main.h"
#include <stdint.h>

/* I2C 句柄由 CubeMX 生成（Core/Src/i2c.c 定义 hi2c1）。
   这里仅 extern 引用；在 CubeMX 启用 I2C1 并重新生成后该符号即存在。
   重复 extern 声明在 C 中合法。 */
extern I2C_HandleTypeDef hi2c1;

/* 设备地址：AD0 接地=0x68，接高=0x69（按你板子改） */
#ifndef MPU6050_I2C_ADDR
#define MPU6050_I2C_ADDR  0x68
#endif

/* 量程灵敏度（必须与 imu_mpu6050.c 中 GYRO/ACCEL_CONFIG 一致） */
#define MPU_ACCEL_LSB_PER_G   16384.0f   /* ±2g  -> 16384 LSB/g */
#define MPU_GYRO_LSB_PER_DPS  131.0f     /* ±250°/s -> 131 LSB/(°/s) */

/* 寄存器地址 */
#define MPU_REG_WHO_AM_I     0x75
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_INT_ENABLE   0x38
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_TEMP_OUT_H   0x41
#define MPU_REG_GYRO_XOUT_H  0x43
#define MPU_REG_PWR_MGMT_2   0x6C

#define MPU_WHO_AM_I_VAL     0x68
#define MPU_WHO_AM_I_VAL_NEW 0x70   /* 部分国产新版本 MPU6050 clone 的 WHO_AM_I */

/* 采样率分频：sample_rate = 1000 / (1 + SMPLRT_DIV)。
   与 attitude.h 的 ATTITUDE_RATE_HZ 保持一致（200Hz => 4）。 */
#ifndef MPU_SMPLRT_DIV
#define MPU_SMPLRT_DIV   4
#endif

int  MPU6050_Init(void);
int  MPU6050_ReadWhoAmI(uint8_t *id);
int  MPU6050_ReadRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
int  MPU6050_EnableInt(void);   /* 使能 data-ready 中断（建议在调度器启动后调用） */
void MPU6050_DumpStatus(void);  /* 诊断：打印 WHO_AM_I/关键寄存器/一次 raw 采样 */
void MPU6050_I2C_BusRecovery(void);  /* I2C 总线锁死恢复（SCL 手动 toggle 释放 SDA）；读失败自动调用 */

#endif /* _IMU_MPU6050_H */
