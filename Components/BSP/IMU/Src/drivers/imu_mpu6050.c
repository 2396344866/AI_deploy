/* =============================================================================
 * MPU6050 底层 I2C 驱动（阶段2：姿态外环之传感器层）
 *   - 仅做寄存器配置 + 原始计数读取，不含任何滤波/融合（那些在 imu_filter / attitude）。
 *   - I2C 用 HAL 轮询（不接 DMA），规避 H7 D-Cache 一致性问题。
 *   - 所有外设句柄/引脚引用 CubeMX 生成宏，本文件位于 Components/，重生成零影响。
 * =============================================================================
 */
#include "imu_mpu6050.h"
#include "logger.h"   /* 诊断打印 LOG_I */

#ifndef MPU_I2C_TIMEOUT
#define MPU_I2C_TIMEOUT  100   /* ms */
#endif

#ifndef MPU_I2C_RECOVERY_RETRIES
#define MPU_I2C_RECOVERY_RETRIES  3   /* 读/写失败后总线恢复并重试次数 */
#endif

/* 薄封装：HAL_I2C_Mem_Read/Write，地址左移 1 位（HAL 要 8 位地址） */
static int I2C_WriteReg(uint8_t reg, uint8_t val)
{
    for (int i = 0; i <= MPU_I2C_RECOVERY_RETRIES; i++) {
        if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(MPU6050_I2C_ADDR << 1),
                reg, I2C_MEMADD_SIZE_8BIT, &val, 1, MPU_I2C_TIMEOUT) == HAL_OK) {
            return 0;
        }
        /* 打印 HAL 错误码定位 Init FAILED 真因（走 logger，受 LOG_ENABLED + 级别闸门）：
           0x04=HAL_I2C_ERROR_AF(NACK，地址/接线/未应答)；0x20=TIMEOUT(总线卡死 SCL/SDA 被拉)；
           0x01=BERR(起始条件冲突)；0x02=ARLO(仲裁丢失)。仅在真实 I2C 失败时触发，正常态零噪音。 */
        LOG_W("MPU", "I2C WR fail reg=0x%02X err=0x%lX (retry %d/%d)",
              (unsigned)reg, (unsigned long)hi2c1.ErrorCode, i, MPU_I2C_RECOVERY_RETRIES);
        MPU6050_I2C_BusRecovery();
    }
    return -1;
}

static int I2C_ReadReg(uint8_t reg, uint8_t *val)
{
    for (int i = 0; i <= MPU_I2C_RECOVERY_RETRIES; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(MPU6050_I2C_ADDR << 1),
                reg, I2C_MEMADD_SIZE_8BIT, val, 1, MPU_I2C_TIMEOUT) == HAL_OK) {
            return 0;
        }
        LOG_W("MPU", "I2C RD fail reg=0x%02X err=0x%lX (retry %d/%d)",
              (unsigned)reg, (unsigned long)hi2c1.ErrorCode, i, MPU_I2C_RECOVERY_RETRIES);
        MPU6050_I2C_BusRecovery();
    }
    return -1;
}

/* 连续读取 N 字节（burst），带总线恢复重试 */
static int I2C_ReadBuf(uint8_t reg, uint8_t *buf, uint16_t len)
{
    for (int i = 0; i <= MPU_I2C_RECOVERY_RETRIES; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(MPU6050_I2C_ADDR << 1),
                reg, I2C_MEMADD_SIZE_8BIT, buf, len, MPU_I2C_TIMEOUT) == HAL_OK) {
            return 0;
        }
        MPU6050_I2C_BusRecovery();
    }
    return -1;
}

/* I2C 总线恢复：总线锁死（SDA 被从机拉低、HAL 在 I2C_IsErrorOccurred 等不到 STOPF）时调用。
   把 PB8(SCL)/PB9(SDA) 临时切 GPIO 开漏，SCL 手动 toggle ≥9 个时钟把从机推出，
   再发 STOP，最后重初始化 I2C1。解决剧烈运动/线束抖动导致的 200Hz 任务卡死。
   引脚与 AF 来自 CubeMX（I2C1=PB8/PB9, AF4），本函数不改任何 CubeMX 生成逻辑。 */
void MPU6050_I2C_BusRecovery(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 1. 关闭 I2C 外设，释放引脚控制 */
    HAL_I2C_DeInit(&hi2c1);

    /* 2. SCL/SDA 切 GPIO 开漏输出，先拉高释放总线 */
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_OUTPUT_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 3. 手动给 SCL 时钟，若 SDA 被从机拉低则逐个时钟推出（经典 9 拍恢复） */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        for (volatile int d = 0; d < 80; d++);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        for (volatile int d = 0; d < 80; d++);
    }

    /* 4. 产生 STOP：SCL 高时 SDA 由低→高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 80; d++);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    for (volatile int d = 0; d < 80; d++);

    /* 5. 重初始化 I2C（HAL_I2C_Init 内部会经 MspInit 把引脚重新配成 AF_OD） */
    HAL_I2C_Init(&hi2c1);
}

int MPU6050_ReadWhoAmI(uint8_t *id)
{
    return I2C_ReadReg(MPU_REG_WHO_AM_I, id);
}

int MPU6050_Init(void)
{
    uint8_t id = 0;
    if (MPU6050_ReadWhoAmI(&id) != 0) {
        /* 器件未应答（I2C NACK）：检查接线 / 地址 / 上拉 / I2C 是否已生成。
           这是唯一真正的“无器件”失败，必须 return -1。 */
        return -1;
    }
    /* 兼容性处理：老版芯片 WHO_AM_I=0x68，部分国产新版本返回 0x70。
       两者寄存器映射与功能完全一致，仅 ID 不同。不再严格比对 0x68，
       避免把 0x70 模块在 Init 阶段直接判死（那样后续唤醒/陀螺配置全不写，
       芯片留在默认 SLEEP 模式，accel/gyro 全读 0）。ID 异常仅告警，仍继续配置。 */
    if (id == MPU_WHO_AM_I_VAL || id == MPU_WHO_AM_I_VAL_NEW) {
        LOG_I("MPU", "WHO_AM_I=0x%02X ok (MPU6050 present)", (unsigned)id);
    } else {
        LOG_W("MPU", "WHO_AM_I=0x%02X unexpected (exp 0x%02X/0x%02X), continue anyway",
              (unsigned)id, (unsigned)MPU_WHO_AM_I_VAL, (unsigned)MPU_WHO_AM_I_VAL_NEW);
    }

    /* 1. 软复位（DEVICE_RESET） */
    I2C_WriteReg(MPU_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    /* 2. 唤醒 + 选 PLL 时钟（CLKSEL=1，陀螺时钟最稳） */
    I2C_WriteReg(MPU_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);
    /* 2a. 显式关闭所有轴的待机（某些 clone 默认不是 0x00），确保 gyro 一定输出 */
    I2C_WriteReg(MPU_REG_PWR_MGMT_2, 0x00);
    /* 3. 采样率分频（决定 data-ready INT 频率） */
    I2C_WriteReg(MPU_REG_SMPLRT_DIV, MPU_SMPLRT_DIV);
    /* 4. DLPF：0x03 => accel 44Hz / gyro 42Hz 低通，抑制高频振动 */
    I2C_WriteReg(MPU_REG_CONFIG, 0x03);
    /* 5. 陀螺量程 ±250°/s */
    I2C_WriteReg(MPU_REG_GYRO_CONFIG, 0x00);
    /* 6. 加速度量程 ±2g */
    I2C_WriteReg(MPU_REG_ACCEL_CONFIG, 0x00);
    /* 7. 注意：data-ready 中断延后到调度器启动后由 MPU6050_EnableInt() 开启，
       避免 200Hz ISR 在 RTOS 内核未起时调用 osSemaphoreRelease（不安全）。 */

    return 0;
}

int MPU6050_EnableInt(void)
{
    /* 使能 data-ready 中断（INT 引脚默认 active-high 推挽脉冲，无需改 INT_PIN_CFG）。
       仅在 RTOS 调度器启动后调用，确保 ISR 内 osSemaphoreRelease 安全。 */
    return I2C_WriteReg(MPU_REG_INT_ENABLE, 0x01);
}

/* 读 accel(6) + gyro(6)，分两次 burst 读取（ACCEL_XOUT_H / GYRO_XOUT_H）。
   某些 MPU6050 clone 在跨 temp 寄存器的 14 字节连续 burst 中 gyro 字节会返回 0；
   分读可绕过该问题，同时仍保留总线恢复重试。 */
int MPU6050_ReadRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t abuf[6], gbuf[6];

    if (I2C_ReadBuf(MPU_REG_ACCEL_XOUT_H, abuf, 6) != 0) {
        return -1;
    }
    if (gyro && I2C_ReadBuf(MPU_REG_GYRO_XOUT_H, gbuf, 6) != 0) {
        return -1;
    }

    accel[0] = (int16_t)((abuf[0] << 8) | abuf[1]);
    accel[1] = (int16_t)((abuf[2] << 8) | abuf[3]);
    accel[2] = (int16_t)((abuf[4] << 8) | abuf[5]);

    if (gyro) {
        gyro[0] = (int16_t)((gbuf[0] << 8) | gbuf[1]);
        gyro[1] = (int16_t)((gbuf[2] << 8) | gbuf[3]);
        gyro[2] = (int16_t)((gbuf[4] << 8) | gbuf[5]);
    }

    if (temp) {
        uint8_t tbuf[2];
        if (I2C_ReadBuf(MPU_REG_TEMP_OUT_H, tbuf, 2) != 0) {
            return -1;
        }
        *temp = (int16_t)((tbuf[0] << 8) | tbuf[1]);
    }
    return 0;
}

/* 诊断：打印 WHO_AM_I、关键寄存器和一次 raw 采样，帮助区分软件配置错误与硬件损坏。
   由串口命令 D 触发（见 Attitude_ProcessCommand），200Hz 任务中不自动调用，零额外开销。 */
void MPU6050_DumpStatus(void)
{
    uint8_t who = 0, pwr1 = 0, pwr2 = 0, gcfg = 0, acfg = 0, cfg = 0;
    int16_t ra[3] = {0}, rg[3] = {0};

    MPU6050_ReadWhoAmI(&who);
    I2C_ReadReg(MPU_REG_PWR_MGMT_1,  &pwr1);
    I2C_ReadReg(MPU_REG_PWR_MGMT_2,  &pwr2);
    I2C_ReadReg(MPU_REG_GYRO_CONFIG, &gcfg);
    I2C_ReadReg(MPU_REG_ACCEL_CONFIG, &acfg);
    I2C_ReadReg(MPU_REG_CONFIG,       &cfg);
    int raw_ok = MPU6050_ReadRaw(ra, rg, NULL);

    LOG_I("MPU", "who=0x%02X pwr1=0x%02X pwr2=0x%02X gcfg=0x%02X acfg=0x%02X cfg=0x%02X | rawA=%d,%d,%d rawG=%d,%d,%d read=%s",
          (unsigned)who, (unsigned)pwr1, (unsigned)pwr2,
          (unsigned)gcfg, (unsigned)acfg, (unsigned)cfg,
          (int)ra[0], (int)ra[1], (int)ra[2],
          (int)rg[0], (int)rg[1], (int)rg[2],
          raw_ok == 0 ? "ok" : "fail");
}
