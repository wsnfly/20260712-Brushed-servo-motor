/*
 * motor_control.h - 直流电机伺服控制器库
 *
 * 功能:
 *   - 位置模式: PID控制电机转到目标位置
 *   - 速度模式: PID控制电机保持目标转速
 *   - 编码器64位扩展计数(通过TIM3更新中断)
 *   - 在线调整PID参数/最大电流
 *
 * 依赖的STM32外设:
 *   TIM1 - 100us定时中断, 内部计数每50次(约5ms)驱动PID运算
 *   TIM3 - 编码器接口模式(PA6=CH1, PA7=CH2)
 *   TIM4 - PWM输出(PB6=CH1, PB7=CH2, PB8=CH3, PB9=CH4)
 *
 * 需要用户实现的中断处理:
 *   TIM1_UP_IRQHandler -> 调用 MotorControl_TimerTick100us()
 *   TIM3_IRQHandler -> 调用 Encoder_OverflowHandler()
 */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"

typedef enum {
    POSITION_MODE = 0,
    SPEED_MODE,
    VELOCITY_POSITION_MODE,  /* 基于位置的速度模式：目标位置按速度累加，走位置PID */
    OPENLOOP_MODE,           /* 开环模式：速度值直接映射到PWM输出 */
    EXTERNAL_TARGET_MODE,    /* 外部目标位置模式：PB4/PB5电平选择目标位置，走位置PID */
    EXTERNAL_TARGET_SPEED_MODE /* 外部目标速度模式：PB4/PB5电平选择目标速度，走速度PID */
} MotorMode_t;

/* 电机方向: CW=正转(默认), CCW=反转 */
typedef enum {
    MOTOR_DIR_CW = 0,   /* 正转 */
    MOTOR_DIR_CCW = 1   /* 反转 */
} MotorDir_t;

/* 编码器方向: NORMAL=正常(默认), REVERSED=反转 */
typedef enum {
    ENCODER_DIR_NORMAL = 0,    /* 正常 */
    ENCODER_DIR_REVERSED = 1   /* 反转 */
} EncoderDir_t;

/* 上电复位模式 */
typedef enum {
    HOME_MODE_OFF = 0,         /* 关闭复位 */
    HOME_MODE_STALL = 1,       /* 堵转检测复位 */
    HOME_MODE_SWITCH = 2       /* 开关精确复位：使用限位开关低速来回检测精确位置 */
} HomeMode_t;

/* PB4/PB5 引脚功能 */
typedef enum {
    PIN_FUNC_PULSE = 0,   /* 脉冲输入：改变目标位置 */
    PIN_FUNC_DIR,          /* 方向信号：脉冲方向控制 */
    PIN_FUNC_HOME,         /* 复位开关：堵转复位时设为原点 */
    PIN_FUNC_LIMIT,        /* 限位开关：停止该方向运动 */
    PIN_FUNC_TARGET,       /* 目标位置/速度电平：外部目标位置/速度模式下有效时到达预设目标位置/速度，无效时回原点/停止 */
    PIN_FUNC_NONE          /* 无功能：引脚不参与任何逻辑 */
} PinFunc_t;

/* 引脚有效电平 */
typedef enum {
    POLARITY_ACTIVE_HIGH = 0,  /* 高电平/上升沿有效 */
    POLARITY_ACTIVE_LOW = 1    /* 低电平/下降沿有效 */
} PinPolarity_t;

#define SPEED_FILTER_SIZE 16  /* 移动平均滤波窗口大小 */

typedef struct {
    float pos_Kp;
    float pos_Ki;
    float pos_Kd;
    float spd_Kp;
    float spd_Ki;
    float spd_Kd;
    int16_t dead_zone;
    int16_t max_output;
    MotorDir_t direction;        /* 电机方向 */
    EncoderDir_t encoder_dir;    /* 编码器方向 */
    uint8_t start_mode;          /* 启动模式: 0=直接启动, 1=需要启动标志位 */
    uint8_t modbus_addr;         /* MODBUS从机地址: 1~247 */
    uint32_t modbus_baud;        /* MODBUS波特率 */
    uint8_t home_mode;             /* 复位模式: 0=关闭, 1=堵转复位, 2=开关精确复位 */
    int8_t home_dir;               /* 复位方向: 0=负方向, 1=正方向 */
    int16_t home_current;          /* 复位电流限制(PWM值) */
    int32_t home_speed;            /* 复位速度(脉冲/秒) */
    int32_t home_max_distance;     /* 复位最大距离(脉冲) */
    int32_t home_back_distance;    /* 碰撞后退回距离(脉冲) */
    uint8_t home_auto_start;       /* 开机自动复位: 0=关闭, 1=开启 */
    int32_t home_precision_speed;  /* 开关精确复位低速检测速度(脉冲/秒) */
    uint8_t home_precision_cycles; /* 开关精确复位来回检测次数 */
    uint8_t pin4_func;           /* PB4功能 (PinFunc_t) */
    uint8_t pin4_polarity;       /* PB4极性 (PinPolarity_t) */
    uint8_t pin4_limit_dir;      /* PB4限位方向: 0=停止正方向, 1=停止负方向 */
    uint8_t pin5_func;           /* PB5功能 (PinFunc_t) */
    uint8_t pin5_polarity;       /* PB5极性 (PinPolarity_t) */
    uint8_t pin5_limit_dir;      /* PB5限位方向: 0=停止正方向, 1=停止负方向 */
    int32_t max_run_speed;       /* 最大运行速度(脉冲/秒), 0=无限制 */
    int64_t pin4_target_pos;     /* PB4目标位置电平功能的预设目标位置 */
    int64_t pin5_target_pos;     /* PB5目标位置电平功能的预设目标位置 */
    int32_t pin4_target_speed;   /* PB4外部目标速度 */
    int32_t pin5_target_speed;   /* PB5外部目标速度 */
    uint32_t tim2_arr;           /* TIM2自动重装载值(ARR), 决定脉冲采样中断周期
                                  * 周期 = (ARR+1)/64MHz, 默认639 -> 10us
                                  * 范围: 99~65535, 即约1.56us~1.024ms */
    MotorMode_t mode;
    float target_speed;
    int64_t target_position;
    float target_pos_frac;  /* 位置速度模式：目标位置小数累加器 */
    float integral;
    float prev_error;
    uint32_t last_tick;
    int64_t prev_position;
    float velocity;
    float prev_velocity;  /* 上一时刻速度, 用于D项 */
    /* 移动平均滤波 */
    float speed_buf[SPEED_FILTER_SIZE];
    uint8_t speed_buf_idx;
    float speed_sum;
    /* PID各项输出(用于调试) */
    float pid_p;
    float pid_i;
    float pid_d;
    float pid_error;
} MotorControl_t;

void MotorControl_Init(MotorControl_t *mc,
                       float pos_Kp, float pos_Ki, float pos_Kd,
                       float spd_Kp, float spd_Ki, float spd_Kd,
                       int16_t dead_zone, int16_t max_output);
void MotorControl_SetTarget(MotorControl_t *mc, int64_t target);
void MotorControl_SetSpeed(MotorControl_t *mc, float speed);
void MotorControl_SetMode(MotorControl_t *mc, MotorMode_t mode);
void MotorControl_SetDirection(MotorControl_t *mc, MotorDir_t dir);
void MotorControl_SetEncoderDirection(MotorControl_t *mc, EncoderDir_t dir);
void MotorControl_SetPosPID(MotorControl_t *mc, float Kp, float Ki, float Kd);
void MotorControl_SetSpdPID(MotorControl_t *mc, float Kp, float Ki, float Kd);
void MotorControl_SetMaxOutput(MotorControl_t *mc, int16_t max_output);
void MotorControl_SetMaxRunSpeed(MotorControl_t *mc, int32_t max_run_speed);
void MotorControl_SetOrigin(MotorControl_t *mc);  /* 设置当前位置为原点（编码器清零+同步目标/积分/速度滤波） */
void MotorControl_StartHoming(MotorControl_t *mc); /* 按配置启动堵转复位，home_mode=0时无动作 */
uint8_t MotorControl_IsHoming(void);
uint8_t MotorControl_HomingFailed(void);
void MotorControl_TimerTick100us(void);  /* 由TIM1 100us中断调用，内部计数每50次执行一次PID */
int16_t MotorControl_Update(MotorControl_t *mc, int64_t current_position);
void MotorControl_Reset(MotorControl_t *mc);
int64_t MotorControl_GetTarget(MotorControl_t *mc);
float MotorControl_GetSpeedTarget(MotorControl_t *mc);
MotorMode_t MotorControl_GetMode(MotorControl_t *mc);
int16_t MotorControl_GetPWM(void);  /* 获取当前PWM输出(-1000~+1000, 即-100%~+100%) */

/* 断电保存/加载配置（PID参数、方向、死区、最大输出、启动模式、MODBUS地址/波特率） */
void MotorControl_SaveConfig(MotorControl_t *mc);
void MotorControl_LoadConfig(MotorControl_t *mc);

void MotorControl_ProcessInputs(void);  /* 主循环中调用, 处理PB4/PB5输入 */
void MotorControl_PulseTick10us(void);  /* TIM2 10us中断中调用, 脉冲边沿检测 */
void MotorControl_UpdateIrqPriority(void);  /* PB4/PB5为脉冲输入时交换TIM1/TIM2优先级，防止丢脉冲 */
void MotorControl_ApplyTim2Arr(void);  /* 应用tim2_arr到TIM2硬件寄存器, 修改脉冲采样中断周期 */
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetPWM(int16_t output);
int64_t Encoder_GetCount(void);
float Encoder_GetSpeed(void);
void Encoder_Reset(void);
void Encoder_OverflowHandler(void);

#endif
