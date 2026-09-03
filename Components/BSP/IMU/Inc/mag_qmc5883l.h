#ifndef _MAG_QMC5883L_H
#define _MAG_QMC5883L_H

#include "main.h"
#include <stdint.h>

/* I2C 句柄由 CubeMX 生成（Core/Src/i2c.c 定义 hi2c1），与 MPU6050 共用 I2C1。
   重复 extern 声明在 C 中合法。 */
extern I2C_HandleTypeDef hi2c1;

/* GY-273 模块多为 QMC5883L，I2C 地址 0x0D；少数批次/兼容脚为 0x1A/0x1C。
   Init 会依次探测候选地址，命中即采用（仿 MPU6050 的容错思路：
   只认“无应答 / 读全 0xFF”为失败，避免硬编码地址把可用模块判死）。 */
#ifndef QMC_I2C_ADDR
#define QMC_I2C_ADDR  0x0D
#endif

int     QMC5822_Init(void);
int     QMC5822_ReadRaw(int16_t mag[3], int block);  /* block!=0 阻塞等锁(Init/诊断/SelfTest)；block==0 抢不到即失败丢帧(Sensor 热路径) */
void    QMC5822_DumpStatus(int block);                /* block 同上；M 命令/诊断传 1 */
uint8_t QMC5822_IsReady(void);   /* 1 = Init 成功探测到芯片 */

#endif /* _MAG_QMC5883L_H */
