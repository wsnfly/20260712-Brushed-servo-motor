/*
 * motor_control.h - 直流电机伺服控制器库
 *
 * 功能:
 *   - 位置模式: PID控制电机转到目标位置
 *   - 速度模式: PID控制电机保持目标转速
 *   - 编码器64位扩展计数(在TIM1 100μs中断中差分累加合成)
 *   - 在线调整PID参数/最大电流
 *
 * 依赖的STM32外设:
 *   TIM1 - 100us定时中断, 每次执行编码器合成, 每50次(约5ms)驱动PID运算
 *   TIM3 - 编码器接口模式(PA6=CH1, PA7=CH2), 溢出中断关闭
 *   TIM4 - PWM输出(PB6=CH1, PB7=CH2, PB8=CH3, PB9=CH4)
 *
 * 需要用户实现的中断处理:
 *   TIM1_UP_IRQHandler -> 调用 MotorControl_TimerTick100us()
 *                         (内部每次调用Encoder_Synthesize做编码器位置合成)
 */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "encoder.h"   /* 编码器模块: Encoder_GetCount/GetSpeed/Reset/Synthesize 等 */

typedef enum {
    POSITION_MODE = 0,
    SPEED_MODE,
    VELOCITY_POSITION_MODE,  /* 基于位置的速度模式：目标位置按速度累加，走位置PID */
    OPENLOOP_MODE,           /* 开环模式：速度值直接映射到PWM输出 */
    EXTERNAL_TARGET_MODE,    /* 外部目标位置模式：PB4/PB5电平选择目标位置，走位置PID */
    EXTERNAL_TARGET_SPEED_MODE, /* 外部目标速度模式：PB4/PB5电平选择目标速度，走速度PID */
    STANDBY_MODE,            /* 待机模式：关闭PWM输出，但保留编码器计数和速度计算 */
    ADC_SPEED_MODE,          /* ADC转速模式：ADC值(0~1)×预设速度, 走速度PID */
    ADC_POSITION_SPEED_MODE, /* ADC位置转速模式：ADC值(0~1)×预设速度, 走位置速度PID */
    ADC_OPENLOOP_MODE,       /* ADC开环模式：ADC值(0~1)×预设PWM, 直接输出 */
    ADC_POSITION_MODE,       /* ADC位置模式：ADC值(0~1)×预设位置, 实时给目标位置走位置PID */
    EXTERNAL_TARGET_POS_SPEED_MODE /* 外部目标位置速度模式：PB4/PB5电平选择目标速度, 位置按速度累加, 走位置PID(无累计误差) */
} MotorMode_t;

/* 判断模式是否基于位置PID (走位置环)
 * 包括: POSITION_MODE, VELOCITY_POSITION_MODE, EXTERNAL_TARGET_MODE, ADC_POSITION_MODE,
 *       EXTERNAL_TARGET_POS_SPEED_MODE
 * 这些模式的目标位置都可通过步进加减速算法平滑 */
static inline uint8_t MotorControl_IsPositionBasedMode(MotorMode_t mode) {
    return (mode == POSITION_MODE || mode == VELOCITY_POSITION_MODE ||
            mode == EXTERNAL_TARGET_MODE || mode == ADC_POSITION_MODE ||
            mode == EXTERNAL_TARGET_POS_SPEED_MODE);
}

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
    HOME_MODE_SWITCH = 2,      /* 开关精确复位：使用限位开关低速来回检测精确位置 */
    HOME_MODE_BIDIR_STALL = 3, /* 双向堵转复位：负方向堵转→正方向堵转→取中点+偏置设原点 */
    HOME_MODE_BIDIR_SENSOR = 4 /* 双向传感器居中复位：负方向限位→正方向限位→取中点+偏置设原点 */
} HomeMode_t;

/* PB4/PB5 引脚功能 */
typedef enum {
    PIN_FUNC_PULSE = 0,        /* 脉冲输入：改变目标位置 */
    PIN_FUNC_DIR,               /* 方向信号：脉冲方向控制 */
    PIN_FUNC_HOME,              /* 原点位置开关：堵转/开关精确复位时检测此信号设定为零点 */
    PIN_FUNC_LIMIT,             /* 限位开关：停止该方向运动 */
    PIN_FUNC_TARGET_POS,        /* 外部目标位置电平：配合EXTERNAL_TARGET_MODE, 有效时到预设目标位置, 无效时回原点 */
    PIN_FUNC_TARGET_SPEED,      /* 外部目标速度电平：配合EXTERNAL_TARGET_SPEED_MODE/EXTERNAL_TARGET_POS_SPEED_MODE, 有效时到预设目标速度, 无效时停止 */
    PIN_FUNC_HOME_START,        /* 执行复位操作：按钮按下时启动一次复位流程(按home_mode配置执行) */
    PIN_FUNC_NONE,              /* 无功能：引脚不参与任何逻辑 */
    PIN_FUNC_STOP,              /* 停止功能：按钮按下时PWM输出为0，松开后恢复 */
    PIN_FUNC_START_FLAG         /* 启动标志位触发：引脚有效边沿触发启动标志位(需start_mode=1), 用于多电机同步启动 */
} PinFunc_t;

/* 引脚有效电平 */
typedef enum {
    POLARITY_ACTIVE_HIGH = 0,  /* 高电平/上升沿有效 */
    POLARITY_ACTIVE_LOW = 1    /* 低电平/下降沿有效 */
} PinPolarity_t;

/* PC2/PC3 ADC 引脚功能 (选择对应电机模式, 或无功能) */
typedef enum {
    ADC_FUNC_NONE = 0,             /* 无功能 */
    ADC_FUNC_SPEED,                /* ADC转速模式 */
    ADC_FUNC_POSITION_SPEED,       /* ADC位置转速模式 */
    ADC_FUNC_OPENLOOP,             /* ADC开环模式 */
    ADC_FUNC_POSITION              /* ADC位置模式 */
} AdcFunc_t;

#define SPEED_FILTER_SIZE 16  /* 移动平均滤波窗口大小 */
#define SPEED_ACQ_BUF_SIZE 5120  /* 采集缓冲区大小(16位寄存器数), 10KB */

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
    uint8_t home_mode;             /* 复位模式: 0=关闭, 1=堵转复位, 2=开关精确复位, 3=双向堵转复位, 4=双向传感器居中复位 */
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
    uint8_t pc2_func;            /* PC2 ADC功能 (AdcFunc_t) */
    uint8_t pc3_func;            /* PC3 ADC功能 (AdcFunc_t) */
    int32_t adc_min_speed;       /* ADC转速/位置转速模式的最小速度(脉冲/秒, 可为负数), ADC值=0时对应此速度 */
    int32_t adc_max_speed;       /* ADC转速/位置转速模式的最大速度(脉冲/秒), ADC值=1时对应此速度 */
    int16_t adc_min_pwm;         /* ADC开环模式的最小PWM(-1000~+1000, 可为负数), ADC值=0时对应此PWM */
    int16_t adc_max_pwm;         /* ADC开环模式的最大PWM(-1000~+1000), ADC值=1时对应此PWM */
    int64_t adc_min_position;    /* ADC位置模式的最小位置(脉冲, 可为负数), ADC值=0时对应此位置 */
    int64_t adc_max_position;    /* ADC位置模式的最大位置(脉冲), ADC值=1时对应此位置 */
    uint8_t  adc_dead_zone1_pos;  /* ADC死区1位置: 0=最小点, 1=中位点, 2=最大点 */
    uint16_t adc_dead_zone1_width;/* ADC死区1宽度(ADC原始值, 0~4095, 0=关闭死区1) */
    uint8_t  adc_dead_zone2_pos;  /* ADC死区2位置: 0=最小点, 1=中位点, 2=最大点 */
    uint16_t adc_dead_zone2_width;/* ADC死区2宽度(ADC原始值, 0~4095, 0=关闭死区2) */

    /* 外部ADC滤波参数(新增)
     * 采用滑动平均(Moving Average), 窗口大小可调:
     *   0或1 = 关闭滤波(直接用原始ADC值)
     *   2~128 = 滑动平均窗口大小, 越大越平滑但滞后越大
     * 主循环5ms周期下延迟对照:
     *   4点≈20ms, 8点≈40ms, 16点≈80ms, 32点≈160ms, 64点≈320ms, 100点≈500ms, 128点≈640ms(最大)
     * 推荐值: 4~32 (兼顾平滑与响应)
     * 算法说明: 维护一个size大小的环形缓冲区, 每次读取ADC求平均.
     *   - 优点: 实现简单, 抗高频噪声效果好, 与电流环滤波风格一致
     *   - 缺点: 对尖峰毛刺抑制一般(若需抗毛刺可后续加中值滤波)
     * 应用位置: ADC_SPEED_MODE/ADC_POSITION_SPEED_MODE/ADC_OPENLOOP_MODE/ADC_POSITION_MODE
     *   模式下, 读取外部ADC后立即滤波, 滤波后值送死区处理和归一化 */
    uint8_t  adc_filter_size;      /* 滤波窗口大小(0或1=关闭, 2~128=滑动平均) */

    /* ========== 步进加减速位置模式（新增）==========
     * 使能后, 位置模式下目标位置不直接给PID, 而是经过步进加减速算法
     * (TIM2中断驱动梯形速度规划)生成平滑的中间位置轨迹dangqianweizhi,
     * 再实时写入电机目标位置, 实现加减速控制.
     * 关闭时保持原行为: 目标位置直接给PID. */
    uint8_t  step_accel_en;        /* 步进加减速使能: 0=关闭(原行为), 1=开启 */
    MotorMode_t mode;
    float target_speed;
    int64_t target_position;
    float target_pos_frac;  /* 位置速度模式/外部目标位置速度模式：目标位置小数累加器 */
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
    /* 速度环PID调试字段(级联模式1ms周期, 新增)
     * 与pid_p/i_d区分: pid_*记录的是位置环输出, spd_loop_*记录速度环输出
     * 上位机可同时读取三环数据, 用于级联调参 */
    float spd_loop_error;       /* 速度环误差 = target_velocity - velocity */
    float spd_loop_p;           /* 速度环P项输出 */
    float spd_loop_i;           /* 速度环I项输出 */
    float spd_loop_d;           /* 速度环D项输出 */
    /* 电流环PID调试字段(100us周期, 新增) */
    float cur_loop_error;       /* 电流环误差 = current_target - current_actual */
    float cur_loop_p;           /* 电流环P项输出 */
    float cur_loop_i;           /* 电流环I项输出 */
    float cur_loop_d;           /* 电流环D项输出 */
    /* 采集缓冲区 (10KB = 5120 个 int16_t, 存储脉冲/秒) */
    int16_t speed_acq_buffer[SPEED_ACQ_BUF_SIZE];
    uint16_t speed_acq_count;      /* 已采集数量 (0~SPEED_ACQ_BUF_SIZE) */
    uint16_t speed_acq_divider;    /* 分频值 (1=100us, 50=5ms, 默认50) */
    uint16_t speed_acq_div_cnt;    /* 分频计数器 (内部使用) */
    uint8_t  speed_acq_active;     /* 采集激活标志 */
    uint8_t  speed_acq_done;       /* 采集完成标志 (采满speed_acq_size个样本) */
    uint8_t  speed_acq_type;       /* 通道1采集类型: 0=转速, 1=PWM, 2=位置, 3=电流, 4=PC0电压ADC, 5=PC2外部ADC, 6=PC3外部ADC,
                                    * 7~15=电流环开启时PID输出(7位置P/8位置I/9位置D/10速度P/11速度I/12速度D/13电流P/14电流I/15电流D),
                                    * 16~21=电流环关闭时PID输出(16位置P/17位置I/18位置D/19速度P/20速度I/21速度D),
                                    * 22=位置误差(目标位置-当前位置), 23=速度误差(目标速度-当前速度) */
    uint8_t  speed_acq_channels;   /* 采集通道数: 1~4, 默认1(单通道兼容原行为).
                                    * 多通道时每个采样周期依次采集所有通道, 缓冲区数据交错存储:
                                    * [ch1_t0, ch2_t0, ..., chN_t0, ch1_t1, ch2_t1, ..., chN_t1, ...]
                                    * 类似STM32 ADC扫描模式, 采样率不变但缓冲区填充快N倍 */
    uint8_t  speed_acq_type2;      /* 通道2采集类型(编码同speed_acq_type, channels>=2时使用) */
    uint8_t  speed_acq_type3;      /* 通道3采集类型(编码同speed_acq_type, channels>=3时使用) */
    uint8_t  speed_acq_type4;      /* 通道4采集类型(编码同speed_acq_type, channels>=4时使用) */
    uint16_t speed_acq_size;       /* 采集总点数 (1~SPEED_ACQ_BUF_SIZE, 默认SPEED_ACQ_BUF_SIZE)
                                    * 多通道时每通道实际点数 = size / channels */
    int64_t  speed_acq_pos_start;  /* 位置采样起始位置(仅type=2使用, 启动采集时记录) */

    /* 堵转保护 */
    uint8_t  stall_protect_en;     /* 堵转保护使能: 0=关闭, 1=开启 */
    int32_t  stall_err_limit;      /* 堵转误差阈值: 位置模式=脉冲, 速度模式=脉冲/秒 */
    uint16_t stall_time_ticks;     /* 堵转持续时间阈值(单位=PID周期5ms, 0=立即触发) */
    uint16_t stall_tick_cnt;       /* 堵转持续计数器(内部使用) */
    uint8_t  stall_tripped;        /* 堵转已触发标志: 0=正常, 1=已触发输出关闭 */

    /* ========== 电流环参数（100μs周期, 新增）==========
     * 电流环使能后, 位置/速度环的输出作为电流目标(相对值-1000~+1000),
     * 电流环根据ADC采样的实际电流做PID, 输出最终PWM.
     * 电流环关闭时, 位置/速度环输出直接作为PWM(原行为).
     *
     * 硬件参考: INA240A1PWR(增益20V/V) + 10mΩ采样电阻
     *   - 总跨阻 = 20 × 0.01 = 0.2 V/A
     *   - 3.3V ADC, 12位, 零电流=1.65V(ADC=2048)
     *   - 满量程电流 = ±1.65V / 0.2 = ±8.25A (对应相对电流 ±1000)
     *   - 1A实际 ≈ 121相对单位 (1A→0.2V→248ADC→121相对)
     *   - 默认 offset=2048, scale=1000/2048≈0.488 (恰好匹配此硬件)
     *   - 过流阈值示例: 5A限流 → over_current_limit=606 (相对值) */
    float cur_Kp;                 /* 电流环比例 */
    float cur_Ki;                 /* 电流环积分 */
    float cur_Kd;                 /* 电流环微分 */
    float cur_Kff;                /* 电流环前馈系数(速度→PWM, 反电动势补偿, 0=关闭)
                                   * PWM += Kff * velocity, 补偿高速反电动势
                                   * 解决限流时电流目标被截顶→P项不足→PWM偏低→速度下降
                                   * 典型值0.015~0.025, ×10000寄存器=150~250 */
    float current_target;         /* 电流目标值(由速度环/位置环输出, -1000~+1000) */
    float current_actual;         /* 实际电流值(ADC标定后, -1000~+1000) */
    float current_integral;       /* 电流环积分项 */
    float current_prev_error;     /* 电流环上次误差(用于D项) */
    uint16_t current_offset;      /* 零电流ADC原始值(标定用, 默认2048=12位ADC中点) */
    float current_scale;          /* ADC原始值到相对电流的转换系数
                                   * 默认 1000.0f/2048 ≈ 0.488
                                   * (ADC值-offset)*scale → 电流相对值 */
    uint8_t  current_loop_en;     /* 电流环使能: 0=关闭(位置/速度环直接输出PWM), 1=开启 */
    uint8_t  over_current_tripped;/* 过流保护已触发: 0=正常, 1=已触发 */
    float over_current_limit;     /* 过流保护阈值(相对值, 0=关闭)
                                   * 实际电流绝对值超过此值时触发保护, 关闭输出 */
    float max_current;            /* 电流限幅(相对值, 0=不限制)
                                   * 电流环运行时电流目标(current_target)绝对值
                                   * 不超过此值, 平滑限幅, 不触发保护.
                                   * 与over_current_limit区别: 限幅限制电流目标,
                                   *   过流保护检测实际电流超限后关闭输出(需复位). */
    float current_dead_zone;      /* 电流死区(相对值, 0=关闭)
                                   * 实际电流绝对值<此值时强制PWM=0, 防止静止时
                                   * 因ADC失调/噪声/H桥漏电流产生细微电流.
                                   * 单位与电流相对值一致(±1000=±8.25A),
                                   * 例: 3≈25mA, 10≈83mA. 0=关闭此功能. */

    /* ========== 电流环使能时的速度环/位置环PID参数（双套PID自动切换）==========
     * 电流环改变被控对象特性: 关闭=电压控制(一阶系统), 使能=电流控制(积分对象).
     * 同一套PID参数无法同时适应两种模式, 因此保存两套参数:
     *   - 电流环关闭: 使用 spd_Kp/Ki/Kd, pos_Kp/Ki/Kd (电压模式参数)
     *   - 电流环使能: 使用 spd_Kp_cur/Ki_cur/Kd_cur, pos_Kp_cur/Ki_cur/Kd_cur (电流模式参数)
     * PID运算时根据current_loop_en自动选择对应参数集.
     * 典型调参: 电流模式Kp约为电压模式的1/3~1/5, Ki约为1/5~1/10 */
    float spd_Kp_cur;             /* 电流环使能时速度环Kp */
    float spd_Ki_cur;             /* 电流环使能时速度环Ki */
    float spd_Kd_cur;             /* 电流环使能时速度环Kd */
    float pos_Kp_cur;             /* 电流环使能时位置环Kp */
    float pos_Ki_cur;             /* 电流环使能时位置环Ki */
    float pos_Kd_cur;             /* 电流环使能时位置环Kd */

    /* ========== 速度环独立周期参数（1ms周期, 新增）==========
     * 级联模式(POSITION_MODE+max_run_speed>0)下, 位置环5ms输出target_velocity,
     * 速度环1ms根据target_velocity做PID输出current_target.
     * 非级联模式下速度环不参与, 位置/速度环直接输出current_target. */
    float target_velocity;        /* 位置环输出的目标速度(级联模式, 脉冲/秒) */
    float speed_loop_integral;    /* 速度环积分项(1ms周期累加) */
    float speed_loop_prev_vel;    /* 速度环上次速度值(用于D项) */
    uint8_t  speed_loop_active;   /* 速度环激活标志(内部使用): 1=级联模式速度环运行中 */

    /* ========== 位置环速度前馈(新增) ==========
     * 步进加减速模式下, 目标位置持续移动. 原D项用-velocity做阻尼,
     * 在目标移动时产生不必要制动力(-234), 积分必须windup到+772来克服,
     * 导致~3秒周期性速度抖动.
     *
     * 修复1: D项改用误差导数 D=Kd*(target_vel-actual_vel), 目标匀速时D≈0
     * 修复2: 显式前馈 FF=pos_Kff*target_vel, 提供维持速度所需的基础输出
     * 效果: 积分不再需要windup, 消除周期性抖动 */
    int64_t prev_target_position; /* 上次目标位置(用于计算目标速度) */
    float   pos_Kff;              /* 位置环速度前馈系数(脉冲/秒→输出, 0=关闭) */
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

/* PB4/PB5输入处理已拆分至 input 模块(见 input.h):
 *   MotorControl_ProcessInputs / MotorControl_UpdateIrqPriority / MotorControl_OnPulseExti
 * 编码器接口已拆分至 encoder 模块(见 encoder.h):
 *   Encoder_GetCount / Encoder_GetSpeed / Encoder_Reset / Encoder_OverflowHandler / Encoder_Synthesize */
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetPWM(int16_t output);

/* 堵转保护 */
uint8_t MotorControl_IsStallTripped(void);          /* 查询是否已触发堵转保护 */
void MotorControl_ResetStall(MotorControl_t *mc);   /* 清除堵转标志并恢复PID运行 */

/* ========== 电流环接口（新增）========== */
void MotorControl_SetCurPID(MotorControl_t *mc, float Kp, float Ki, float Kd);
void MotorControl_EnableCurrentLoop(MotorControl_t *mc, uint8_t en);  /* 0=关闭, 1=开启 */
void MotorControl_SetCurrentCalib(MotorControl_t *mc, uint16_t offset, float scale);  /* 电流标定 */
void MotorControl_SetOverCurrentLimit(MotorControl_t *mc, float limit);  /* 过流阈值, 0=关闭 */
void MotorControl_SetMaxCurrent(MotorControl_t *mc, float max_current);  /* 电流限幅, 0=不限制 */
void MotorControl_SetCurrentDeadZone(MotorControl_t *mc, float dead_zone);  /* 电流死区, 0=关闭 */
uint8_t MotorControl_IsOverCurrentTripped(void);
void MotorControl_ResetOverCurrent(MotorControl_t *mc);
float MotorControl_GetCurrent(void);        /* 获取实际电流(相对值) */
float MotorControl_GetCurrentTarget(void);  /* 获取电流目标(相对值) */
float MotorControl_GetSupplyVoltage(void);  /* 获取供电电压(V), PC0分压100K:10K */
uint16_t MotorControl_GetSupplyVoltageADC(void);  /* 获取供电电压ADC原始值(0~4095, PC0) */
uint16_t MotorControl_GetExternalADC(uint8_t channel);  /* 外部ADC原始值, 0=PC2, 1=PC3 */
uint16_t MotorControl_GetExternalADCFiltered(MotorControl_t *mc, uint8_t channel);  /* 外部ADC滤波后值(受adc_filter_size控制) */
uint16_t MotorControl_GetCurrentADC(void);  /* 获取电流ADC原始值(0~4095, PC1) */
uint16_t MotorControl_AutoCalibrateCurrentOffset(MotorControl_t *mc);  /* 自动零点标定, 返回新offset */

#endif
