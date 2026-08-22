/* =============================================================================
 * GY-273 / QMC5883L 三轴磁力计驱动（I2C1，与 MPU6050 共用总线）
 * -----------------------------------------------------------------------------
 * 用途：阶段1 9轴融合的磁力计数据源。本文件只负责"芯片探测 + 原始计数读取 +
 *       诊断 dump"，不做标定/融合（标定留阶段2 持久化，融合在 attitude.c）。
 *
 * 接线（GY-273 模块，已接）：
 *   VCC -> 3.3V   GND -> GND   SCL -> PB8(I2C1)   SDA -> PB9(I2C1)
 *   模块自带上拉，与 MPU6050 同挂 I2C1（7位地址 0x0D）。
 *
 * 注意：QMC5883L 与 HMC5883L 寄存器不兼容，本驱动专攻 QMC5883L。
 *   若手持模块读 WHO 寄存器异常，优先用 M 命令看 DumpStatus 的 raw 值，
 *   只要 raw 非全 0xFF 即通信正常，可忽略 WHO 探测。
 * ============================================================================= */
#include "mag_qmc5883l.h"
#include "logger.h"
#include <string.h>

#ifndef QMC_I2C_ADDR
#define QMC_I2C_ADDR  0x0D
#endif

/* 寄存器地址（QMC5883L） */
#define QMC_REG_DATA      0x00   /* X_L,X_H,Y_L,Y_H,Z_L,Z_H 连续 6 字节 */
#define QMC_REG_CTRL2     0x0B   /* 控制寄存器2：0x00 即可（无中断/无软复位） */
#define QMC_REG_RST_PERIOD 0x0A  /* SET/RESET 周期：写 0x01 */
#define QMC_REG_CTRL1     0x09   /* 控制寄存器1：模式/ODR/量程/过采样 */
#define QMC_REG_CHIP_ID   0x0D   /* 芯片ID（部分批次读 0xFF，仅作参考） */

/* CTRL1 位域（连续测量模式 / 200Hz / 2G / OSR=512）：0x1D */
#define QMC_CTRL1_VAL     0x1D
/* 2G 满量程对应的计数（用于 counts->mG 转换，标定前仅参考）：约 12000 counts */
#define QMC_RANGE_2G_COUNTS  12000.0f

static uint8_t s_ready = 0;
static int16_t s_last_raw[3] = {0, 0, 0};

/* 薄封装：HAL I2C，地址左移 1 位（HAL 要 8 位地址） */
static int qmc_write(uint8_t reg, uint8_t val)
{
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(QMC_I2C_ADDR << 1),
                          reg, 1, &val, 1, 10) != HAL_OK) {
        return -1;
    }
    return 0;
}

static int qmc_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(QMC_I2C_ADDR << 1),
                         reg, 1, buf, len, 10) != HAL_OK) {
        return -1;
    }
    return 0;
}

int QMC5822_Init(void)
{
    s_ready = 0;
    uint8_t who = 0;
    /* WHO 寄存器仅作信息打印（某些批次为 0xFF），不以它判生死 */
    (void)qmc_read(QMC_REG_CHIP_ID, &who, 1);

    /* 上电稳定后初始化序列 */
    HAL_Delay(10);
    if (qmc_write(QMC_REG_RST_PERIOD, 0x01) != 0) {
        LOG_W("MAG", "QMC write RST_PERIOD fail (no ACK? addr=0x%02X)", (unsigned)QMC_I2C_ADDR);
        return -1;
    }
    if (qmc_write(QMC_REG_CTRL2, 0x00) != 0) {
        LOG_W("MAG", "QMC write CTRL2 fail");
        return -1;
    }
    if (qmc_write(QMC_REG_CTRL1, QMC_CTRL1_VAL) != 0) {
        LOG_W("MAG", "QMC write CTRL1 fail");
        return -1;
    }
    HAL_Delay(10);

    /* 探测：读一次数据，非全 0xFF 即认为芯片在线 */
    uint8_t tmp[6];
    if (qmc_read(QMC_REG_DATA, tmp, 6) != 0) {
        LOG_W("MAG", "QMC read data fail (check wiring/I2C addr)");
        return -1;
    }
    int all_ff = 1;
    for (int i = 0; i < 6; i++) if (tmp[i] != 0xFF) { all_ff = 0; break; }
    if (all_ff) {
        LOG_W("MAG", "QMC data all 0xFF -> not ready");
        return -1;
    }

    s_ready = 1;
    LOG_I("MAG", "QMC5883L ready (who=0x%02X addr=0x%02X, 2G=%d counts)",
          (unsigned)who, (unsigned)QMC_I2C_ADDR, (int)QMC_RANGE_2G_COUNTS);
    return 0;
}

int QMC5822_ReadRaw(int16_t mag[3])
{
    if (!s_ready) return -1;
    uint8_t buf[6];
    if (qmc_read(QMC_REG_DATA, buf, 6) != 0) return -1;
    /* 小端：低字节在前 */
    int16_t x = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    int16_t y = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    int16_t z = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    mag[0] = x; mag[1] = y; mag[2] = z;
    s_last_raw[0] = x; s_last_raw[1] = y; s_last_raw[2] = z;
    return 0;
}

uint8_t QMC5822_IsReady(void) { return s_ready; }

void QMC5822_DumpStatus(void)
{
    uint8_t ctrl1 = 0, ctrl2 = 0, rsta = 0, who = 0;
    (void)qmc_read(QMC_REG_CTRL1, &ctrl1, 1);
    (void)qmc_read(QMC_REG_CTRL2, &ctrl2, 1);
    (void)qmc_read(QMC_REG_RST_PERIOD, &rsta, 1);
    (void)qmc_read(QMC_REG_CHIP_ID, &who, 1);
    LOG_I("MAG", "ready=%u who=0x%02X ctrl1=0x%02X ctrl2=0x%02X rst=0x%02X",
          (unsigned)s_ready, (unsigned)who, (unsigned)ctrl1,
          (unsigned)ctrl2, (unsigned)rsta);
    LOG_I("MAG", "raw(X,Y,Z)=%d,%d,%d (counts)  range_2G=%.0f counts",
          (int)s_last_raw[0], (int)s_last_raw[1], (int)s_last_raw[2],
          (double)QMC_RANGE_2G_COUNTS);
}
