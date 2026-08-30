/* =============================================================================
 * 双电机 速度/位置闭环 实现（阶段1）
 * 参考：大鱼电子 PID 速度/位置闭环例程（增量式 PI）
 * =============================================================================
 */
#include "motor.h"
#include <stdlib.h>   /* atoi */
#include <string.h>   /* memcpy */

/* -----------------------------------------------------------------------------
 * 运行态定义
 * --------------------------------------------------------------------------- */
static Motor_t g_motor[MOTOR_B + 1] = {
    /* 电机 A：TIM1_CH1(PA8) + TIM4(PB6/PB7) 编码器，方向 PB0/PB1 */
    {
        .id = MOTOR_A,
        .htim_pwm = &htim1, .pwm_ch = TIM_CHANNEL_1,
        .htim_enc = &htim4,
        .ain1_port = TB6612_AIN1_GPIO_Port, .ain1_pin = TB6612_AIN1_Pin,
        .ain2_port = TB6612_AIN2_GPIO_Port, .ain2_pin = TB6612_AIN2_Pin,
        .enc_sign = 1,
        .cmd_dir  = 1,
    },
    /* 电机 B：TIM1_CH2(PE11) + TIM3(PB4/PB5) 编码器，方向 PB2/PB3 */
    {
        .id = MOTOR_B,
        .htim_pwm = &htim1, .pwm_ch = TIM_CHANNEL_2,
        .htim_enc = &htim3,
        .ain1_port = TB6612_BIN1_GPIO_Port, .ain1_pin = TB6612_BIN1_Pin,
        .ain2_port = TB6612_BIN2_GPIO_Port, .ain2_pin = TB6612_BIN2_Pin,
        .enc_sign = 1,
        .cmd_dir  = -1,
    },
};

MotorSys_t g_motor_sys = { .running = 0, .mode = 0, .tick = 0 };

/* 无仪表探针：在 1ms 节拍累计 PWMA(PA8)/PWMB(PE11) 高电平样本数。
   注意：1kHz 采样 × 10kHz PWM 是整数倍，会严重混叠，这里的 hi/ticks 只能
   用来判断“引脚有没有电平变化/有没有被驱动”，不能当作真实占空比。
   真实占空比请直接读取 TIM1->CCR1/CCR2。 */
static uint32_t s_probe_ticks = 0;
static uint32_t s_probe_hi[2] = { 0, 0 };   /* [0]=PA8(PWMA), [1]=PE11(PWMB) */

/* TIM1 计数器窗口内 min/max：若 TIM1 在计数，1 秒内会遍历 0..ARR，span≈ARR；
   若冻结（CNT 恒为 0），span≈0。用于无仪表确认 PWM 定时器是否真的在跑。 */
static uint32_t s_cnt_min = 0xFFFFFFFFU;
static uint32_t s_cnt_max = 0U;

/* -----------------------------------------------------------------------------
 * 内部：编码器读取（16-bit 有符号，防溢出，累计到 int32）
 *   直接读 CNT（short 强转自动处理 0xFFFF->-1 的回绕），再清零。
 *   与 CubeMX 配置一致：编码器中断未使能，由控制环 1ms 轮询读取。
 * --------------------------------------------------------------------------- */
static int16_t Encoder_Read(TIM_HandleTypeDef *htim)
{
    int16_t cnt = (int16_t)(htim->Instance->CNT);
    __HAL_TIM_SET_COUNTER(htim, 0);
    return cnt;
}

/* -----------------------------------------------------------------------------
 * 内部：设置单电机方向 + 占空比
 *   pwm 为带符号整数（-PWM_MAX ~ +PWM_MAX）：
 *     正 -> AIN1=1/AIN2=0（正转）；负 -> AIN1=0/AIN2=1（反转）；0 -> 刹车
 *   CCR 只接受正值（绝对值）。
 * --------------------------------------------------------------------------- */
static void Motor_SetDirPwm(Motor_t *m, int32_t pwm)
{
    int32_t mag = pwm;
    if (mag < 0) mag = -mag;
    if (mag > PWM_MAX) mag = PWM_MAX;

    if (pwm > 0) {                       /* 正转 */
        HAL_GPIO_WritePin(m->ain1_port, m->ain1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->ain2_port, m->ain2_pin, GPIO_PIN_RESET);
    } else if (pwm < 0) {                /* 反转 */
        HAL_GPIO_WritePin(m->ain1_port, m->ain1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->ain2_port, m->ain2_pin, GPIO_PIN_SET);
    } else {                             /* 刹车（同电平） */
        HAL_GPIO_WritePin(m->ain1_port, m->ain1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->ain2_port, m->ain2_pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_ch, (uint32_t)mag);
}

/* -----------------------------------------------------------------------------
 * 增量式 PI（速度环）
 *   与 大鱼电子 例程同构：
 *     Bias = Target - Encoder   （目标减实测，实测偏慢时 Bias>0）
 *     Pwm += Kp*(Bias - Last_bias) + Ki*Bias
 *   直接在 float 域累加，输出限幅到 [-PWM_MAX, +PWM_MAX]。
 * --------------------------------------------------------------------------- */
static int32_t Incremental_PI(Motor_t *m, int32_t enc, int32_t target)
{
    int32_t bias = target - enc;           /* 偏差=目标-实测：实测偏慢时 Bias>0，PWM 增大 -> 趋近目标 */
    int32_t d = bias - m->pid_bias_last;
    m->pid_bias_last = bias;

    /* 用浮点 PI 累加（保证平滑），限幅到 [-PWM_MAX, +PWM_MAX] */
    m->pid_pwm += VELOCITY_KP * (float)d + VELOCITY_KI * (float)bias;

    if (m->pid_pwm > (float)PWM_MAX)  m->pid_pwm = (float)PWM_MAX;
    if (m->pid_pwm < -(float)PWM_MAX) m->pid_pwm = -(float)PWM_MAX;

    return (int32_t)m->pid_pwm;
}

/* -----------------------------------------------------------------------------
 * 单电机一步控制（速度环 / 位置环级联）
 *   速度模式：target_speed 直接作为速度环目标。
 *   位置模式：位置偏差经 POSITION_KP 得到速度目标（限幅 TARGET_SPEED_MAX），再进速度环。
 * --------------------------------------------------------------------------- */
void Motor_ControlStep(Motor_t *m)
{
    /* 1. 读编码器（原始增量 + 累计），按每电机 enc_sign 修正计数方向。
       两个电机 enc_sign 均为 +1：+pwm 时编码器读数增大，保证 (命令pwm)
       与 (实测speed) 同号。B 的方向一致性由 cmd_dir 在输出级处理。 */
    int32_t raw_delta = (int32_t)Encoder_Read(m->htim_enc);
    int32_t enc_delta = raw_delta * (int32_t)m->enc_sign;
    m->enc_total += enc_delta;
    m->speed = enc_delta;                 /* 速度 = 节拍内计数增量 */
    m->position += enc_delta;

    /* 2. 目标选择 */
    int32_t speed_target = m->target_speed;
    if (g_motor_sys.mode == 1) {          /* 位置模式：位置环 -> 速度目标 */
        int32_t pos_err = m->target_pos - m->position;
        if (pos_err >  POSITION_MAX) pos_err =  POSITION_MAX;
        if (pos_err < -POSITION_MAX) pos_err = -POSITION_MAX;
        speed_target = (int32_t)((float)pos_err * POSITION_KP);
        if (speed_target >  TARGET_SPEED_MAX) speed_target =  TARGET_SPEED_MAX;
        if (speed_target < -TARGET_SPEED_MAX) speed_target = -TARGET_SPEED_MAX;
    }

    /* 3. 速度环 PI -> PWM（带符号） */
    int32_t pid = Incremental_PI(m, m->speed, speed_target);

    /* 目标速度为 0 时：积分缓慢泄放，确保完全刹车。
       增量式 PI 在 bias=0 时会「保持上一拍 pwm」，导致 target=0 刹不住；
       这里直接把 pid_pwm 向 0 指数泄放，彻底消除该隐患（A/B 通用）。 */
    if (speed_target == 0) {
        m->pid_pwm *= 0.85f;
        pid = (int32_t)m->pid_pwm;
    }

    /* 应用层方向符号：A=+1（与电机同向）；B=-1（中心对称：B 的 +命令
       对应与 A 相反的轮向）。cmd_dir 翻转后，B 的增量式 PI 仍是负反馈
       （+pwm->raw 为负已被 cmd_dir=-1 抵消），既能收敛也保留中心对称。 */
    m->pwm_out = (int32_t)m->cmd_dir * pid;

    /* 4. 输出（停机则强制刹车） */
    if (g_motor_sys.running) {
        Motor_SetDirPwm(m, m->pwm_out);
    } else {
        Motor_SetDirPwm(m, 0);
    }
}

/* -----------------------------------------------------------------------------
 * 1ms 控制节拍（在 TIM7 中断 USER CODE 中调用）
 * --------------------------------------------------------------------------- */
void Motor_1ms_Handler(void)
{
    g_motor_sys.tick++;
    Motor_ControlStep(&g_motor[MOTOR_A]);
    Motor_ControlStep(&g_motor[MOTOR_B]);

    /* 探针：累计 PWMA/PWMB 引脚高电平样本（见文件顶部说明） */
    s_probe_ticks++;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  == GPIO_PIN_SET) s_probe_hi[0]++;
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET) s_probe_hi[1]++;

    /* 探针：采样 TIM1 计数器，判断 PWM 定时器是否真的在计数（冻结则 span≈0） */
    uint32_t c = htim1.Instance->CNT;
    if (c < s_cnt_min) s_cnt_min = c;
    if (c > s_cnt_max) s_cnt_max = c;
}

/* 读取并清零探针计数；hiA/hiB 返回高电平样本数，ticks 返回总样本数（≈1000/秒）。
   占空比 ≈ hiA / ticks。STBY 由调用方读取。 */
void Motor_Probe_ReadAndClear(uint32_t *hiA, uint32_t *hiB, uint32_t *ticks)
{
    if (hiA)   *hiA   = s_probe_hi[0];
    if (hiB)   *hiB   = s_probe_hi[1];
    if (ticks) *ticks = s_probe_ticks;
    s_probe_hi[0] = 0; s_probe_hi[1] = 0; s_probe_ticks = 0;
}

/* 返回上一窗口内 TIM1 计数器跨度（max-min）；≈ARR(999) 表示在计数，≈0 表示冻结。
   读取后清零。 */
uint32_t Motor_Probe_Tim1Span(void)
{
    uint32_t span = (s_cnt_max >= s_cnt_min) ? (s_cnt_max - s_cnt_min) : 0U;
    s_cnt_min = 0xFFFFFFFFU;
    s_cnt_max = 0U;
    return span;
}

/* -----------------------------------------------------------------------------
 * 应用初始化：启动 PWM / 编码器计数 / TIM7 中断
 * --------------------------------------------------------------------------- */
void Motor_App_Init(void)
{
    /* 拉高 STBY（已在 CubeMX 置高，这里确保一次） */
    HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);

    /* 编码器：启动计数（CubeMX 已 Init，但 HAL 编码器需 Start） */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* PWM：启动 CH1/CH2 输出（占空比 0） */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* 防御性兜底：高级定时器 TIM1 必须 CEN+MOE 同时置位才会计数并输出。
       若 HAL_TIM_PWM_Start 因句柄状态等原因未真正生效，这里强制开启。 */
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_ENABLE(&htim1);

    /* TIM7 1ms 定时中断（CubeMX 已 Enable IRQ + SetPriority） */
    HAL_TIM_Base_Start_IT(&htim7);

    /* 默认停机（刹车），等待 KEY 或命令启动 */
    g_motor_sys.running = 0;
    g_motor_sys.mode = 0;

    LOG_I("MOTOR", "Motor subsystem inited: TIM1 10kHz PWM, TIM3/4 enc(x4), TIM7 1ms");
}

/* -----------------------------------------------------------------------------
 * 命令接口
 * --------------------------------------------------------------------------- */
void Motor_SetSpeed(MotorID id, int32_t speed)
{
    if (id > MOTOR_B) return;
    g_motor[id].target_speed = speed;
    g_motor_sys.mode = 0;
}

void Motor_SetPosition(MotorID id, int32_t pos)
{
    if (id > MOTOR_B) return;
    g_motor[id].target_pos = pos;
    g_motor_sys.mode = 1;
}

void Motor_EmergencyStop(void)
{
    g_motor_sys.running = 0;
    Motor_SetDirPwm(&g_motor[MOTOR_A], 0);
    Motor_SetDirPwm(&g_motor[MOTOR_B], 0);
    LOG_W("MOTOR", "Emergency stop -> brake");
}

void Motor_Resume(void)
{
    g_motor_sys.running = 1;
    LOG_I("MOTOR", "Resume running");
}

int32_t Motor_GetSpeed(MotorID id)
{
    if (id > MOTOR_B) return 0;
    return g_motor[id].speed;
}

int32_t Motor_GetPosition(MotorID id)
{
    if (id > MOTOR_B) return 0;
    return g_motor[id].position;
}

/* 转速估算：speed(计数/节拍) * 1000(节拍/s) / ENCODER_CPR(计数/圈) * 60(s/min) */
float Motor_GetRPM(MotorID id)
{
    if (id > MOTOR_B) return 0.0f;
    float rev_per_sec = (float)g_motor[id].speed * (float)MOTOR_CTL_HZ / (float)ENCODER_CPR;
    return rev_per_sec * 60.0f;
}

/* 当前输出 PWM（带符号，绝对值送 CCR；+ 正转 / - 反转 / 0 刹车）。
   用于联调时确认固件确实在向 TB6612 命令占空比——若 pwm≠0 而电机不动，必为硬件问题。 */
int32_t Motor_GetPWM(MotorID id)
{
    if (id > MOTOR_B) return 0;
    return g_motor[id].pwm_out;
}

int32_t Motor_GetTargetSpeed(MotorID id)
{
    if (id > MOTOR_B) return 0;
    return g_motor[id].target_speed;
}

/* -----------------------------------------------------------------------------
 * 串口命令解析（由 BSP_LOG 的 OnFrame 帧回调调用）
 *   帧格式：以 IDLE 中断收到的一帧文本，例如 "A200\r\n"
 *   协议：
 *     A<速度>  设电机A目标速度（速度模式），如 A200 / A-150
 *     B<速度>  设电机B目标速度（速度模式），如 B200
 *     S        急停（刹车并停机）
 *     R        恢复运行
 *   设速度时若当前未运行，自动 Resume，方便无仪表调试。
 * --------------------------------------------------------------------------- */
void Motor_ProcessCommand(const char *cmd, uint16_t len)
{
    if (cmd == NULL || len == 0U) return;

    /* 复制到局部缓冲并补 '\0'，atoi 需要以 null 结尾的字符串 */
    char buf[32];
    uint16_t n = (len < (sizeof(buf) - 1U)) ? len : (sizeof(buf) - 1U);
    memcpy(buf, cmd, n);
    buf[n] = '\0';

    char op = buf[0];
    int32_t val = 0;
    if (op == 'A' || op == 'B') {
        val = (int32_t)atoi(&buf[1]);
    }

    switch (op) {
        case 'A':
            Motor_SetSpeed(MOTOR_A, val);
            if (!g_motor_sys.running) Motor_Resume();
            LOG_I("MOTOR", "A target=%ld (speed mode)", val);
            break;
        case 'B':
            Motor_SetSpeed(MOTOR_B, val);
            if (!g_motor_sys.running) Motor_Resume();
            LOG_I("MOTOR", "B target=%ld (speed mode)", val);
            break;
        case 'S':
            Motor_EmergencyStop();
            break;
        case 'R':
            Motor_Resume();
            break;
        default:
            LOG_W("MOTOR", "Unknown cmd '%c' (use A200/B200/S/R)", op);
            break;
    }
}
