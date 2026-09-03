#ifndef _MOTOR_H_
#define _MOTOR_H_

/* =============================================================================
 * 双电机 速度/位置闭环 组件（阶段1）
 *   - TB6612FNG 驱动：TIM1_CH1(PA8)/CH2(PE11) PWM + AIN1/AIN2/BIN1/BIN2/PB12(STBY)
 *   - 编码器：TIM4(电机A: PB6/PB7) / TIM3(电机B: PB4/PB5)，正交 ×4 倍频
 *   - 控制环：TIM7 1ms 节拍（在 stm32h7xx_it.c 的 TIM7_IRQHandler 中调用）
 *   - 算法：增量式 PI 速度环 + 位置环级联（对齐 大鱼电子 例程）
 *
 * 设计原则：
 *   - 所有引脚只引用 CubeMX 生成的宏（main.h 的 xxx_Pin / xxx_GPIO_Port）
 *     与定时器句柄（tim.h 的 htim1/htim3/htim4/htim7），不硬编码任何引脚号。
 *   - 自定义代码放在 Components/，不被 CubeMX 重新生成覆盖。
 * =============================================================================
 */

#include "main.h"
#include "tim.h"
#include "logger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 控制节拍 ---------- */
#define MOTOR_CTL_DT_MS   1U      /* TIM7 = 1ms */
#define MOTOR_CTL_HZ      1000U   /* 1000 Hz */

/* ---------- PWM 分辨率（= TIM1 ARR = 1000-1） ---------- */
#define PWM_MAX           999     /* 满占空比对应 CCR 值，必须 ≤ ARR；999/1000 ≈ 99.9% */

/* ---------- 编码器（4x 倍频） ---------- */
/* TT 带编码电机常见 PPR=11 -> 单圈 44 计数；按实物铭牌/实测校准 */
#ifndef ENCODER_PPR
#define ENCODER_PPR       11
#endif
#define ENCODER_CPR       (ENCODER_PPR * 4)   /* counts per revolution */

/* 编码器 16-bit 计数器半量程，超过即判定反向溢出 */
#define ENC_HALF          (32768)

/* ---------- PID 参数（待整定） ---------- */
#ifndef VELOCITY_KP
#define VELOCITY_KP       14.0f
#endif
#ifndef VELOCITY_KI
#define VELOCITY_KI       8.0f
#endif
#ifndef POSITION_KP
#define POSITION_KP       0.04f
#endif
#define TARGET_SPEED_MAX  200     /* 位置环输出（速度目标）的限幅，计数/节拍 */
#define POSITION_MAX      (ENCODER_CPR * 20) /* 位置目标限幅，防止积分跑飞 */

/* ---------- 电机编号 ---------- */
typedef enum {
    MOTOR_A = 0,   /* 电机 A：TIM1_CH1(PA8) + TIM4 编码器 */
    MOTOR_B = 1    /* 电机 B：TIM1_CH2(PE11) + TIM3 编码器 */
} MotorID;

/* ---------- 每电机运行态 ---------- */
typedef struct {
    MotorID   id;
    TIM_HandleTypeDef *htim_pwm;   /* TIM1 */
    uint32_t  pwm_ch;              /* TIM_CHANNEL_1 / _2 */
    TIM_HandleTypeDef *htim_enc;   /* TIM3 / TIM4 */
    GPIO_TypeDef *ain1_port; uint32_t ain1_pin;
    GPIO_TypeDef *ain2_port; uint32_t ain2_pin;

    /* 编码器原始计数方向符号：+1 表示 +pwm 时编码器读数增大。
       两个电机都保持 +1（B 的方向一致性由 cmd_dir 处理，不在这里翻）。 */
    int8_t    enc_sign;

    /* 应用层命令方向符号：作用于「PI 输出 -> 实际施加 pwm」。
       A = +1（与电机同向，+命令=前进）；
       B = -1（中心对称：B 的 +命令 对应与 A 相反的轮向，即右轮后退）。
       数学上闭环稳定要求 cmd_dir 与「+pwm->raw 增」同号；实测 B 的
       +pwm -> raw 为负，故 B 必须 = -1，否则增量式 PI 正反馈发散、刹不住。 */
    int8_t    cmd_dir;

    /* 运行数据 */
    int32_t   enc_total;     /* 累计计数（多圈） */
    int16_t   enc_last;      /* 上次原始 CNT */
    int32_t   speed;         /* 速度（计数/节拍），+ 正转 */
    int32_t   position;      /* 位置（累计计数） */

    /* 控制目标 */
    int32_t   target_speed;  /* 速度环目标（计数/节拍） */
    int32_t   target_pos;    /* 位置环目标（累计计数） */

    /* 速度环 PID（增量式） */
    float     pid_pwm;       /* 积分累加的 PWM（带符号） */
    int32_t   pid_bias_last; /* 上次偏差 */

    /* 输出（带符号 PWM，绝对值送 CCR） */
    int32_t   pwm_out;
} Motor_t;

/* ---------- 全局使能 / 运行状态 ---------- */
typedef struct {
    uint8_t   running;       /* 0=停机（刹车），1=运行 */
    uint8_t   mode;          /* 0=速度模式，1=位置模式 */
    uint32_t  tick;          /* 控制节拍计数 */
} MotorSys_t;

extern MotorSys_t g_motor_sys;

/* ---------- API ---------- */
void     Motor_App_Init(void);          /* 启动 PWM/编码器/TIM7，拉高 STBY */
void     Motor_1ms_Handler(void);       /* 1ms 控制节拍（TIM7 中断调用） */

void     Motor_SetSpeed(MotorID id, int32_t speed);   /* 速度模式目标 */
void     Motor_SetPosition(MotorID id, int32_t pos);  /* 位置模式目标 */
void     Motor_EmergencyStop(void);                    /* 刹车并停机 */
void     Motor_Resume(void);                            /* 解除刹车，恢复运行 */

/* 串口命令解析（由 BSP_LOG 的 OnFrame 帧回调调用）
   协议：A<速度> 设电机A目标(速度模式)  B<速度> 设电机B目标
         S 急停(刹车)  R 恢复运行 */
void     Motor_ProcessCommand(const char *cmd, uint16_t len);

int32_t  Motor_GetSpeed(MotorID id);     /* 当前速度（计数/节拍） */
int32_t  Motor_GetPosition(MotorID id);  /* 当前累计位置 */
float    Motor_GetRPM(MotorID id);       /* 估算转速 RPM（需 ENCODER_PPR 准确） */
int32_t  Motor_GetPWM(MotorID id);       /* 当前输出 PWM（带符号，绝对值送 CCR） */
int32_t  Motor_GetTargetSpeed(MotorID id); /* 当前速度环目标（计数/节拍） */

/* 无仪表探针：读取并清零 PWMA/PWMB 引脚高电平累计（见 motor.c）。
   hiA/hiB 高电平样本数，ticks 总样本数（≈1000/秒）；占空比≈hiA/ticks。 */
void     Motor_Probe_ReadAndClear(uint32_t *hiA, uint32_t *hiB, uint32_t *ticks);

/* 返回上一窗口内 TIM1 计数器跨度（max-min）：≈ARR(999) 表示 PWM 定时器在计数，
   ≈0 表示冻结（CNT 卡死，PWM 输出被钉在比较态，电机无法受控）。 */
uint32_t  Motor_Probe_Tim1Span(void);

/* POST 电机自检入口（按 APP_ENABLE_MOTOR 门控）：外设链路存活校验（不实际转动）。
 * 实现见 motor.c 尾部。 */
int Motor_Test(void);

/* 内部：单电机一步控制（速度环 / 位置环级联） */
void     Motor_ControlStep(Motor_t *m);

#ifdef __cplusplus
}
#endif

#endif /* _MOTOR_H_ */
