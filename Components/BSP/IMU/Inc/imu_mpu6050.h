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
#define MPU_REG_USER_CTRL    0x6A   /* 用户控制：bit5=I2C_MST_EN（关掉才能让主机直通 aux bus） */
#define MPU_REG_INT_PIN_CFG  0x37   /* 中断/旁路配置：bit1=I2C_BYPASS_EN（主总线直通 XCL/XDA） */
#define MPU_I2C_MST_EN       0x20
#define MPU_I2C_BYPASS_EN    0x02

#define MPU_WHO_AM_I_VAL     0x68
#define MPU_WHO_AM_I_VAL_NEW 0x70   /* 部分国产新版本 MPU6050 clone 的 WHO_AM_I */

/* 采样率分频：sample_rate = 1000 / (1 + SMPLRT_DIV)。
   与 attitude.h 的 ATTITUDE_RATE_HZ 保持一致（200Hz => 4）。 */
#ifndef MPU_SMPLRT_DIV
#define MPU_SMPLRT_DIV   4
#endif

int  MPU6050_Init(void);
int  MPU6050_ReadWhoAmI(uint8_t *id);
int  MPU6050_IsBypassEnabled(void);  /* 回读 INT_PIN_CFG[I2C_BYPASS_EN]，确认 aux bus 旁路生效（QMC 可达性），供 POSTEST 诊断 */
void MPU6050_ScanBus(void);           /* 诊断：扫描 I2C 总线并打出所有 ACK 的从机地址，定位 aux 上有无 QMC */
int  MPU6050_ReadRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp, int block);  /* block!=0 阻塞等锁(标定/Init/诊断)；block==0 抢不到即失败丢帧(Sensor 热路径) */
int  MPU6050_EnableInt(void);   /* 使能 data-ready 中断（建议在调度器启动后调用） */
void MPU6050_DumpStatus(void);  /* 诊断：打印 WHO_AM_I/关键寄存器/一次 raw 采样 */
void MPU6050_I2C_BusRecovery(void);  /* I2C 总线锁死恢复（SCL 手动 toggle 释放 SDA）；读失败自动调用 */

/* I2C1 总线互斥锁接口（锁对象由 CubeMX 生成：Middleware→FREERTOS→Mutexes 添加 i2c1_mutex → i2c1_mutexHandle）。
   所有 hi2c1 访问（MPU6050 + QMC5883L）必须用本接口包住，防止 Sensor 任务(AboveNormal,200Hz) 与
   控制台 C/M 命令(Logger,Low2) 争用同一总线，导致 C 标定时大量读失败(gyro cal skipped) 与
   BusRecovery 把共享句柄 DeInit/ReInit 拆掉。详见 Components/Debug/Error/sensor_error.md E42。
   - block!=0：阻塞等锁(osWaitForever)，用于标定/Init/诊断等必须完成的路径；
   - block==0：0 超时，用于 Sensor 热路径，抢不到锁即返回失败、由调用方丢一帧（不阻塞、不反转优先级）；
   - 内核未运行(预调度期)或锁未创建：直接返回 0(未加锁)，单线程无争用，无需锁。
   注：BusRecovery 只在已持锁的事务内调用，自身不再加锁。 */
int  MPU6050_I2C_Lock(int block);     /* 返回 1=已加锁, 0=未加锁(预调度/抢不到) */
void MPU6050_I2C_Unlock(int locked); /* locked==1 才释放，与 Lock 配对 */

#endif /* _IMU_MPU6050_H */
