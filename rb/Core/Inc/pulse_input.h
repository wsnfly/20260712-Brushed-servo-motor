/**
 * @file pulse_input.h
 * @brief 脉冲输入模块 - PA15/PB3 硬件计数与逻辑功能切换
 *
 * 当引脚功能配置为 PIN_FUNC_PULSE 时, 启用TIM2编码器模式硬件计数:
 *   - PA15 = TIM2_CH1 (脉冲输入, TI1边沿计数)
 *   - PB3  = TIM2_CH2 (方向输入, TI2电平决定计数方向)
 *   - TIM2工作在Encoder Mode TI1, 硬件自动计数, 不占CPU
 *   - 64位位置合成: TIM1 100μs中断里差分累加TIM2->CNT (与TIM3编码器同机制)
 *   - 不开启TIM2任何中断, 完全由TIM1 100μs中断主动采样
 *
 * 当引脚功能为其他(STOP/HOME_START/START_FLAG/DIR)时:
 *   - PA15/PB3 配置为普通GPIO输入
 *   - 直接读IDR寄存器获取电平, input.c中处理逻辑功能
 *
 * 设计参考: encoder.c的64位合成机制, 完全复用同一套差分累加算法.
 */
#ifndef __PULSE_INPUT_H
#define __PULSE_INPUT_H

#include "motor_control.h"

/* 初始化脉冲输入模块: 绑定电机控制实例指针.
 * 由 MotorControl_Init 在加载配置后调用, 根据pin4/pin5_func决定是否启动TIM2 */
void PulseInput_Init(MotorControl_t *mc);

/* 根据pin4/pin5_func配置PA15/PB3工作模式:
 *   - 若任一引脚为PIN_FUNC_PULSE: 配置为AF输入, 启动TIM2编码器模式
 *   - 否则: 配置为普通GPIO输入(交由input.c的EXTI/轮询处理)
 * 由 MotorControl_UpdateIrqPriority 调用(配置变更时) */
void PulseInput_Reconfig(MotorControl_t *mc);

/* TIM2脉冲位置合成: 在TIM1 100μs中断中调用, 差分累加TIM2->CNT到64位
 * 算法与Encoder_Synthesize完全一致: delta = (int16_t)(cnt_curr - cnt_prev) */
void PulseInput_Synthesize(void);

/* 消费脉冲累加值: 读取并清零64位脉冲计数差分.
 * 在MotorControl_Update中调用, 把脉冲增量累加到target_position.
 * 返回值: 自上次调用以来的脉冲累计增量(带方向, 正负)
 * 注意: 内部关全局中断保证64位原子读, 关中断时间极短(几条指令) */
int32_t PulseInput_ConsumeDelta(MotorControl_t *mc);

/* 启动/停止TIM2编码器(由Motor_Start/Motor_Stop调用, 与TIM3同步启停) */
void PulseInput_Start(void);
void PulseInput_Stop(void);

/* 获取当前TIM2脉冲累计值(64位, 仅供调试/状态查询用) */
int64_t PulseInput_GetCount(void);

/* 复位脉冲计数器(清零CNT/prev/total, 用于回零/校准) */
void PulseInput_Reset(void);

#endif /* __PULSE_INPUT_H */
