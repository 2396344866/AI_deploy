/* =============================================================================
 * MPU6050 底层 I2C 驱动（阶段2：姿态外环之传感器层）
 *   - 仅做寄存器配置 + 原始计数读取，不含任何滤波/融合（那些在 imu_filter / attitude）。
 *   - I2C 用 HAL 轮询（不接 DMA），规避 H7 D-Cache 一致性问题。
 *   - 所有外设句柄/引脚引用 CubeMX 生成宏，本文件位于 Components/，重生成零影响。
 * =============================================================================
 */
#include "imu_mpu6050.h"
#include "logger.h"   /* 诊断打印 LOG_I */
#include "cmsis_os2.h"   /* osMutexId_t / osMutexAcquire / osMutexRelease / osKernelGetState（CubeMX 生成的 i2c1_mutexHandle 类型与锁 API） */

#ifndef MPU_I2C_TIMEOUT
#define MPU_I2C_TIMEOUT  100   /* ms */
#endif

#ifndef MPU_I2C_RECOVERY_RETRIES
#define MPU_I2C_RECOVERY_RETRIES  3   /* 读/写失败后总线恢复并重试次数 */
#endif

/* I2C1 总线互斥锁（锁对象 CubeMX 生成：i2c1_mutexHandle）。
   所有 hi2c1 访问必须包在 MPU6050_I2C_Lock/Unlock 之间，避免 Sensor(AboveNormal,200Hz) 与
   控制台 C/M 命令(Logger,Low2) 争用同一总线导致 C 标定时大量读失败(gyro cal skipped) 与
   BusRecovery 拆共享句柄。详见 Components/Debug/Error/sensor_error.md E42。 */
extern osMutexId_t i2c1_mutexHandle;

int MPU6050_I2C_Lock(int block)
{
    /* 内核未运行（预调度期 Init 路径）或锁未创建 → 单线程无争用，直接放行 */
    if (osKernelGetState() != osKernelRunning) return 0;
    if (i2c1_mutexHandle == NULL)            return 0;
    /* block!=0 阻塞等锁(osWaitForever)；block==0 仅尝试一次(0 超时)，抢不到即失败由调用方丢帧 */
    uint32_t to = block ? osWaitForever : 0U;
    return (osMutexAcquire(i2c1_mutexHandle, to) == osOK) ? 1 : 0;
}

void MPU6050_I2C_Unlock(int locked)
{
    if (locked && i2c1_mutexHandle != NULL) {
        osMutexRelease(i2c1_mutexHandle);
    }
}

/* 薄封装：HAL_I2C_Mem_Read/Write，地址左移 1 位（HAL 要 8 位地址） */
static int I2C_WriteReg(uint8_t reg, uint8_t val, int block)
{
    int locked = MPU6050_I2C_Lock(block);
    if (!locked) return -1;   /* Sensor 0 超时抢不到锁 → 直接失败，由调用方丢帧 */
    int rc = -1;
    for (int i = 0; i <= MPU_I2C_RECOVERY_RETRIES; i++) {
        if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(MPU6050_I2C_ADDR << 1),
                reg, I2C_MEMADD_SIZE_8BIT, &val, 1, MPU_I2C_TIMEOUT) == HAL_OK) {
            rc = 0; break;
        }
        /* 打印 HAL 错误码定位 Init FAILED 真因（走 logger，受 LOG_ENABLED + 级别闸门）：
           0x04=HAL_I2C_ERROR_AF(NACK，地址/接线/未应答)；0x20=TIMEOUT(总线卡死 SCL/SDA 被拉)；
           0x01=BERR(起始条件冲突)；0x02=ARLO(仲裁丢失)。仅在真实 I2C 失败时触发，正常态零噪音。 */
        LOG_W("MPU", "I2C WR fail reg=0x%02X err=0x%lX (retry %d/%d)",
              (unsigned)reg, (unsigned long)hi2c1.ErrorCode, i, MPU_I2C_RECOVERY_RETRIES);
        MPU6050_I2C_BusRecovery();   /* 已持锁，内部不再加锁 */
    }
    MPU6050_I2C_Unlock(locked);
    return rc;
}

static int I2C_ReadReg(uint8_t reg, uint8_t *val, int block)
{
    int locked = MPU6050_I2C_Lock(block);
    if (!locked) return -1;
    int rc = -1;
    for (int i = 0; i <= MPU_I2C_RECOVERY_RETRIES; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(MPU6050_I2C_ADDR << 1),
                reg, I2C_MEMADD_SIZE_8BIT, val, 1, MPU_I2C_TIMEOUT) == HAL_OK) {
            rc = 0; break;
        }
        LOG_W("MPU", "I2C RD fail reg=0x%02X err=0x%lX (retry %d/%d)",
              (unsigned)reg, (unsigned long)hi2c1.ErrorCode, i, MPU_I2C_RECOVERY_RETRIES);
        MPU6050_I2C_BusRecovery();
    }
    MPU6050_I2C_Unlock(locked);
    return rc;
}

/* 连续读取 N 字节（burst），带总线恢复重试。block 语义同 I2C_WriteReg/ReadReg：
   Sensor 热路径传 0（抢不到锁即失败丢帧），Init/诊断/标定传 1（阻塞等锁跑完）。 */
static int I2C_ReadBuf(uint8_t reg, uint8_t *buf, uint16_t len, int block)
{
    int locked = MPU6050_I2C_Lock(block);
    if (!locked) return -1;   /* Sensor 0 超时抢不到锁 → 直接失败，由调用方丢帧 */
    int rc = -1;
    for (int i = 0; i <= MPU_I2C_RECOVERY_RETRIES; i++) {
        if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(MPU6050_I2C_ADDR << 1),
                reg, I2C_MEMADD_SIZE_8BIT, buf, len, MPU_I2C_TIMEOUT) == HAL_OK) {
            rc = 0; break;
        }
        MPU6050_I2C_BusRecovery();   /* 已持锁，内部不再加锁 */
    }
    MPU6050_I2C_Unlock(locked);
    return rc;
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
    return I2C_ReadReg(MPU_REG_WHO_AM_I, id, 1);   /* 诊断/Init 路径，阻塞等锁 */
}

int MPU6050_IsBypassEnabled(void)
{
    uint8_t v = 0;
    if (I2C_ReadReg(MPU_REG_INT_PIN_CFG, &v, 1) != 0) return -1;   /* 诊断/Init 路径，阻塞等锁 */
    return (v & MPU_I2C_BYPASS_EN) ? 1 : 0;
}

/* I2C 总线扫描（诊断用）：逐个 7 位地址发 dummy write，记录 ACK 的从机。
   用于定位"aux 总线上到底有哪些器件"——若仅 0x68(MPU) 而无 0x0D(QMC)，
   说明 MPU6050 旁路未把 XDA/XCL 透传到主总线，或 QMC 实际未接在 aux 上。
   仅在 Sensor_Test 的 QMC 未就绪分支调用（POSTEST 期 IWDG 已 disarm，无需喂狗）。 */
void MPU6050_ScanBus(void)
{
    LOG_I("MAG", "I2C bus scan start (hi2c1, 0x08..0x77) ...");
    int found = 0;
    /* 整个扫描期间持锁（block=1），避免与 Sensor/C/M 命令交错损坏总线。
       扫描为诊断用途（Sensor_Test 期 IWDG 已 disarm），无实时性压力。 */
    int locked = MPU6050_I2C_Lock(1);
    for (uint16_t a = 0x08; a <= 0x77; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 3) == HAL_OK) {
            LOG_I("MAG", "  ACK @ 0x%02X", (unsigned)a);
            found++;
        }
    }
    MPU6050_I2C_Unlock(locked);
    LOG_I("MAG", "I2C bus scan done: %d device(s) responded", found);
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
    I2C_WriteReg(MPU_REG_PWR_MGMT_1, 0x80, 1);
    HAL_Delay(100);
    /* 2. 唤醒 + 选 PLL 时钟（CLKSEL=1，陀螺时钟最稳） */
    I2C_WriteReg(MPU_REG_PWR_MGMT_1, 0x01, 1);
    HAL_Delay(10);
    /* 2a. 显式关闭所有轴的待机（某些 clone 默认不是 0x00），确保 gyro 一定输出 */
    I2C_WriteReg(MPU_REG_PWR_MGMT_2, 0x00, 1);
    /* 3. 采样率分频（决定 data-ready INT 频率） */
    I2C_WriteReg(MPU_REG_SMPLRT_DIV, MPU_SMPLRT_DIV, 1);
    /* 4. DLPF：0x03 => accel 44Hz / gyro 42Hz 低通，抑制高频振动 */
    I2C_WriteReg(MPU_REG_CONFIG, 0x03, 1);
    /* 5. 陀螺量程 ±250°/s */
    I2C_WriteReg(MPU_REG_GYRO_CONFIG, 0x00, 1);
    /* 6. 加速度量程 ±2g */
    I2C_WriteReg(MPU_REG_ACCEL_CONFIG, 0x00, 1);
    /* 7. 注意：data-ready 中断延后到调度器启动后由 MPU6050_EnableInt() 开启，
       避免 200Hz ISR 在 RTOS 内核未起时调用 osSemaphoreRelease（不安全）。 */

    /* 8. QMC5883L 物理接在 MPU6050 XCL/XDA（auxiliary I2C bus）。开启 I2C bypass：
       先关 MPU 内部 I2C master（USER_CTRL[I2C_MST_EN]=0），再置 INT_PIN_CFG[I2C_BYPASS_EN]=1，
       让 STM32 主机经 PB8/PB9 直通访问 aux bus 上的 QMC（地址仍 0x0D）。
       这样 mag_qmc5883l.c 无需改动即可找到芯片。 */
    I2C_WriteReg(MPU_REG_USER_CTRL,   0x00, 1);   /* 关 MPU 内部 I2C master：释放 aux bus 给主机 */
    I2C_WriteReg(MPU_REG_INT_PIN_CFG, MPU_I2C_BYPASS_EN, 1);  /* bit1=1：主总线(PB8/PB9) 直通 XCL/XDA */
    HAL_Delay(5);

    /* 回读校验：旁路是否真的生效（部分 clone 需顺序/时序，未生效则 aux bus 上 QMC 不可达）。 */
    uint8_t pin_cfg = 0;
    if (I2C_ReadReg(MPU_REG_INT_PIN_CFG, &pin_cfg, 1) == 0) {
        if (pin_cfg & MPU_I2C_BYPASS_EN)
            LOG_I("MPU", "I2C bypass ENABLED (INT_PIN_CFG=0x%02X): QMC@aux reachable via PB8/PB9",
                  (unsigned)pin_cfg);
        else
            LOG_W("MPU", "I2C bypass NOT set (INT_PIN_CFG=0x%02X)! QMC on aux bus unreachable",
                  (unsigned)pin_cfg);
    } else {
        LOG_W("MPU", "I2C bypass read-back NACK (INT_PIN_CFG unreadable)");
    }

    return 0;
}

int MPU6050_EnableInt(void)
{
    /* 使能 data-ready 中断（INT 引脚默认 active-high 推挽脉冲，无需改 INT_PIN_CFG）。
       仅在 RTOS 调度器启动后调用，确保 ISR 内 osSemaphoreRelease 安全。 */
    return I2C_WriteReg(MPU_REG_INT_ENABLE, 0x01, 1);
}

/* 读 accel(6) + gyro(6)，分两次 burst 读取（ACCEL_XOUT_H / GYRO_XOUT_H）。
   某些 MPU6050 clone 在跨 temp 寄存器的 14 字节连续 burst 中 gyro 字节会返回 0；
   分读可绕过该问题，同时仍保留总线恢复重试。 */
int MPU6050_ReadRaw(int16_t accel[3], int16_t gyro[3], int16_t *temp, int block)
{
    uint8_t abuf[6], gbuf[6];

    if (I2C_ReadBuf(MPU_REG_ACCEL_XOUT_H, abuf, 6, block) != 0) {
        return -1;
    }
    if (gyro && I2C_ReadBuf(MPU_REG_GYRO_XOUT_H, gbuf, 6, block) != 0) {
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
        if (I2C_ReadBuf(MPU_REG_TEMP_OUT_H, tbuf, 2, block) != 0) {
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
    I2C_ReadReg(MPU_REG_PWR_MGMT_1,  &pwr1, 1);
    I2C_ReadReg(MPU_REG_PWR_MGMT_2,  &pwr2, 1);
    I2C_ReadReg(MPU_REG_GYRO_CONFIG, &gcfg, 1);
    I2C_ReadReg(MPU_REG_ACCEL_CONFIG, &acfg, 1);
    I2C_ReadReg(MPU_REG_CONFIG,       &cfg, 1);
    int raw_ok = MPU6050_ReadRaw(ra, rg, NULL, 1);

    LOG_I("MPU", "who=0x%02X pwr1=0x%02X pwr2=0x%02X gcfg=0x%02X acfg=0x%02X cfg=0x%02X | rawA=%d,%d,%d rawG=%d,%d,%d read=%s",
          (unsigned)who, (unsigned)pwr1, (unsigned)pwr2,
          (unsigned)gcfg, (unsigned)acfg, (unsigned)cfg,
          (int)ra[0], (int)ra[1], (int)ra[2],
          (int)rg[0], (int)rg[1], (int)rg[2],
          raw_ok == 0 ? "ok" : "fail");
}
