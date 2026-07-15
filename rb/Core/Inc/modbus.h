/**
  ******************************************************************************
  * @file    modbus.h
  * @brief   MODBUS RTU协议头文件
  ******************************************************************************
  */

#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"
#include "motor_control.h"

/* MODBUS功能码 */
#define MB_FUNC_READ_HOLDING_REGS    0x03  /* 读保持寄存器 */
#define MB_FUNC_WRITE_SINGLE_REG     0x06  /* 写单个寄存器 */
#define MB_FUNC_WRITE_MULTIPLE_REGS  0x10  /* 写多个寄存器 */

/* MODBUS异常码 */
#define MB_EX_NONE                   0x00
#define MB_EX_ILLEGAL_FUNCTION       0x01
#define MB_EX_ILLEGAL_DATA_ADDRESS   0x02
#define MB_EX_ILLEGAL_DATA_VALUE     0x03
#define MB_EX_SLAVE_DEVICE_FAILURE   0x04

/* 寄存器地址定义 */
/* 控制寄存器 (读写) */
#define REG_CONTROL_WORD             0x0000  /* 模式设置 */
#define REG_MODE_SET                 0x0001  /* 启动方式 (0=直接启动, 1=标志位启动) */
#define REG_TARGET_POS_H3            0x0002  /* 目标位置高32位 */
#define REG_TARGET_POS_H2            0x0003  /* 目标位置高16位 */
#define REG_TARGET_POS_L2            0x0004  /* 目标位置低16位 */
#define REG_TARGET_POS_L1            0x0005  /* 目标位置最低位 */
#define REG_TARGET_SPEED_H           0x0006  /* 目标速度高16位 */
#define REG_TARGET_SPEED_L           0x0007  /* 目标速度低16位 */
#define REG_POS_KP                   0x0008  /* 位置环Kp (×100) */
#define REG_POS_KI                   0x0009  /* 位置环Ki (×100) */
#define REG_POS_KD                   0x000A  /* 位置环Kd (×100) */
#define REG_SPD_KP                   0x000B  /* 速度环Kp (×100) */
#define REG_SPD_KI                   0x000C  /* 速度环Ki (×100) */
#define REG_SPD_KD                   0x000D  /* 速度环Kd (×100) */
#define REG_DEAD_ZONE                0x000E  /* 死区 */
#define REG_MAX_OUTPUT               0x000F  /* 最大输出 */
#define REG_START_MODE               0x0010  /* 启动触发 (写1启动，仅在0x0001=1时需要) */
#define REG_DIRECTION                0x0011  /* 电机方向 (0=CW正转, 1=CCW反转) */
#define REG_ENCODER_DIRECTION        0x0012  /* 编码器方向 (0=正常, 1=反转) */
#define REG_START_FLAG               0x0013  /* 保留寄存器 */
#define REG_SLAVE_ADDR               0x0014  /* MODBUS从机地址 (1~247) */
#define REG_BAUD_RATE                0x0015  /* MODBUS波特率 (波特率/100) */
#define REG_SET_ORIGIN               0x0016  /* 设置当前位置为原点 (写1触发, 读取始终返回0) */
#define REG_HOME_MODE                0x0017  /* 复位模式 (0=关闭, 1=堵转复位) */
#define REG_HOME_DIRECTION           0x0018  /* 复位方向 (0=负方向, 1=正方向) */
#define REG_HOME_CURRENT             0x0019  /* 复位电流限制(PWM值, 10~1000) */
#define REG_HOME_SPEED_H             0x001A  /* 复位速度高16位 */
#define REG_HOME_SPEED_L             0x001B  /* 复位速度低16位 */
#define REG_HOME_MAX_DISTANCE_H      0x001C  /* 复位最大距离高16位 */
#define REG_HOME_MAX_DISTANCE_L      0x001D  /* 复位最大距离低16位 */
#define REG_HOME_BACK_DISTANCE_H     0x001E  /* 碰撞后退回距离高16位 */
#define REG_HOME_BACK_DISTANCE_L     0x001F  /* 碰撞后退回距离低16位 */
#define REG_HOME_TRIGGER             0x0020  /* 执行一次复位 (写1触发, 读取始终返回0) */
#define REG_HOME_AUTO_START          0x0021  /* 开机自动复位 (0=关闭, 1=开启, 默认1) */
#define REG_INPUT1_FUNC              0x0022  /* PB4功能 (0=脉冲, 1=方向, 2=复位, 3=限位, 4=目标位置电平) */
#define REG_INPUT1_POLARITY          0x0023  /* PB4极性 (0=高电平有效, 1=低电平有效) */
#define REG_INPUT1_LIMIT_DIR         0x0024  /* PB4限位方向 (0=停止正方向, 1=停止负方向) */
#define REG_INPUT2_FUNC              0x0025  /* PB5功能 */
#define REG_INPUT2_POLARITY          0x0026  /* PB5极性 */
#define REG_INPUT2_LIMIT_DIR         0x0027  /* PB5限位方向 */
#define REG_MAX_RUN_SPEED_H          0x0028  /* 最大运行速度高16位(脉冲/秒, 0=无限制) */
#define REG_MAX_RUN_SPEED_L          0x0029  /* 最大运行速度低16位 */
#define REG_INPUT1_TARGET_POS_H3     0x002A  /* PB4目标位置电平预设位置 bit[63:48] */
#define REG_INPUT1_TARGET_POS_H2     0x002B  /* PB4目标位置电平预设位置 bit[47:32] */
#define REG_INPUT1_TARGET_POS_L2     0x002C  /* PB4目标位置电平预设位置 bit[31:16] */
#define REG_INPUT1_TARGET_POS_L1     0x002D  /* PB4目标位置电平预设位置 bit[15:0] */
#define REG_INPUT2_TARGET_POS_H3     0x002E  /* PB5目标位置电平预设位置 bit[63:48] */
#define REG_INPUT2_TARGET_POS_H2     0x002F  /* PB5目标位置电平预设位置 bit[47:32] */
#define REG_INPUT2_TARGET_POS_L2     0x0030  /* PB5目标位置电平预设位置 bit[31:16] */
#define REG_INPUT2_TARGET_POS_L1     0x0031  /* PB5目标位置电平预设位置 bit[15:0] */
#define REG_HOME_PRECISION_SPEED_H   0x0032  /* 开关精确复位低速检测速度高16位 */
#define REG_HOME_PRECISION_SPEED_L   0x0033  /* 开关精确复位低速检测速度低16位 */
#define REG_HOME_PRECISION_CYCLES    0x0034  /* 开关精确复位来回检测次数 */
#define REG_INPUT1_TARGET_SPEED_H    0x0035  /* PB4外部目标速度高16位 */
#define REG_INPUT1_TARGET_SPEED_L    0x0036  /* PB4外部目标速度低16位 */
#define REG_INPUT2_TARGET_SPEED_H    0x0037  /* PB5外部目标速度高16位 */
#define REG_INPUT2_TARGET_SPEED_L    0x0038  /* PB5外部目标速度低16位 */
#define REG_TIM2_ARR                 0x0039  /* TIM2自动重装载值(ARR), 决定脉冲采样中断周期
                                             * 周期 = (ARR+1)/64MHz, 默认639 -> 10us
                                             * 范围: 99~65535, 即约1.56us~1.024ms
                                             * 增大ARR可降低中断频率以减轻CPU负担(影响脉冲采样精度) */
#define REG_SPEED_ACQ_START          0x003A  /* 转速采集启动/状态: 写1启动采集, 读=状态(0=空闲,1=采集中,2=完成) */
#define REG_SPEED_ACQ_DIV            0x003B  /* 转速采集分频值(1=100us, 50=5ms, 默认50) */
#define REG_SPEED_ACQ_COUNT          0x003C  /* 采集已采样数量(只读, 0~5120) */
#define REG_SPEED_ACQ_STATUS         0x003D  /* 转速采集状态(只读, 同0x003A读取值) */
#define REG_SPEED_ACQ_TYPE           0x003E  /* 采集类型: 0=转速(脉冲/秒), 1=PWM输出(-1000~+1000) */
#define REG_SPEED_ACQ_SIZE           0x003F  /* 采集点数(1~5120, 默认5120) */

/* 数据寄存器 (只读) - 采集缓冲区 0x0200~0x15FF (5120个int16_t, 10KB) */
#define REG_SPEED_DATA_BASE          0x0200  /* 采集数据起始地址 */
#define REG_SPEED_DATA_END           0x15FF  /* 采集数据结束地址 (0x0200 + SPEED_ACQ_BUF_SIZE - 1) */

/* 状态寄存器 (只读) */
#define REG_CURRENT_POS_H3           0x0100  /* 当前位置高32位 */
#define REG_CURRENT_POS_H2           0x0101  /* 当前位置高16位 */
#define REG_CURRENT_POS_L2           0x0102  /* 当前位置低16位 */
#define REG_CURRENT_POS_L1           0x0103  /* 当前位置最低位 */
#define REG_CURRENT_SPEED_H          0x0104  /* 当前速度高16位 */
#define REG_CURRENT_SPEED_L          0x0105  /* 当前速度低16位 */
#define REG_CURRENT_MODE             0x0106  /* 当前模式 */
#define REG_STATUS                   0x0107  /* 状态字 */
#define REG_CURRENT_PWM              0x0108  /* 当前PWM输出(有符号, -1000~+1000, 即±100%) */
#define REG_PID_ERROR_H              0x0109  /* PID当前误差高16位(×100, 有符号32位) */
#define REG_PID_ERROR_L              0x010A  /* PID当前误差低16位 */
#define REG_PID_P_H                  0x010B  /* PID比例项P输出高16位(×100) */
#define REG_PID_P_L                  0x010C  /* PID比例项P输出低16位 */
#define REG_PID_I_H                  0x010D  /* PID积分项I输出高16位(×100) */
#define REG_PID_I_L                  0x010E  /* PID积分项I输出低16位 */
#define REG_PID_D_H                  0x010F  /* PID微分项D输出高16位(×100) */
#define REG_PID_D_L                  0x0110  /* PID微分项D输出低16位 */

/* 控制字位定义 */
#define CTRL_MODE_MASK               0x000F  /* 模式掩码 */

/* 模式定义 (与motor_control.h的MotorMode_t对应) */
#define MODE_POSITION                0  /* 位置模式 (POSITION_MODE) */
#define MODE_SPEED                   1  /* 速度模式 (SPEED_MODE) */
#define MODE_VELOCITY_POSITION       2  /* 位置速度模式 (VELOCITY_POSITION_MODE) */
#define MODE_OPENLOOP                3  /* 开环模式 (OPENLOOP_MODE) */
#define MODE_EXTERNAL_TARGET         4  /* 外部目标位置模式 (EXTERNAL_TARGET_MODE) */
#define MODE_EXTERNAL_TARGET_SPEED   5  /* 外部目标速度模式 (EXTERNAL_TARGET_SPEED_MODE) */

/* MODBUS接收缓冲区大小 */
#define MB_RX_BUF_SIZE               128
#define MB_TX_BUF_SIZE               256  /* 支持一次读取最多125个寄存器(255字节) */

/*
 * 帧间超时最小阈值（TIM1 100us中断计数）
 * 实际阈值随当前波特率按3.5字符时间动态计算，且不低于4个100us计数
 */
#define MB_FRAME_TIMEOUT_TICKS       4

/* MODBUS上下文 */
typedef struct {
    uint8_t rx_buf[MB_RX_BUF_SIZE];          /* 接收缓冲区（中断中逐字节填充） */
    uint8_t rx_process_buf[MB_RX_BUF_SIZE];  /* 处理缓冲区（主循环使用） */
    uint8_t rx_byte;                          /* 单字节接收缓冲（HAL用） */
    volatile uint16_t rx_index;              /* 当前接收字节索引 */
    volatile uint16_t rx_len;                /* 接收到的数据长度 */
    volatile uint16_t timeout_ticks;         /* 帧间超时计数（TIM1每100us+1） */
    volatile uint16_t frame_timeout_ticks;   /* 帧间超时阈值（随波特率调整） */
    volatile uint8_t rx_complete;            /* 帧接收完成标志 */
    uint8_t tx_buf[MB_TX_BUF_SIZE];
    uint8_t slave_addr;
    MotorControl_t *motor;

    /* 调试统计 */
    volatile uint32_t rx_count;      /* 接收帧计数 */
    volatile uint32_t error_count;   /* 错误计数 */
} MB_Context_t;

/* 系统时间（100µs单位，TIM1中断中累加） */
extern volatile int64_t sys_time_100us;

/* 获取系统时间（原子读取） */
static inline int64_t SYS_GetTime100us(void) {
    int64_t t;
    __disable_irq();
    t = sys_time_100us;
    __enable_irq();
    return t;
}

/* 获取系统时间（ms单位） */
static inline int64_t SYS_GetTimeMs(void) {
    return SYS_GetTime100us() / 10;
}

/* 函数声明 */
void MODBUS_Init(MB_Context_t *ctx, uint8_t slave_addr, MotorControl_t *motor);
void MODBUS_OnByteReceived(MB_Context_t *ctx, uint8_t byte);   /* UART中断中调用 */
void MODBUS_OnTimeoutTick(MB_Context_t *ctx);                  /* TIM1中断中调用 */
void MODBUS_Process(MB_Context_t *ctx);                        /* 主循环中调用 */
void MODBUS_SendResponse(MB_Context_t *ctx, uint8_t *data, uint8_t len);
uint8_t MODBUS_IsSupportedBaud(uint32_t baud_rate);
uint16_t MODBUS_EncodeBaud(uint32_t baud_rate);
uint32_t MODBUS_DecodeBaud(uint16_t value);
uint16_t MODBUS_GetFrameTimeoutTicks(uint32_t baud_rate);
HAL_StatusTypeDef MODBUS_ApplyBaudRate(MB_Context_t *ctx, uint32_t baud_rate);
void MODBUS_SyncTargetPosition(int64_t position);  /* 外部模块同步目标位置（脉冲输入等） */

#endif /* __MODBUS_H */
