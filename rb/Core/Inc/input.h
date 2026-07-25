/**
 * @file input.h
 * @brief 输入模块 - PB4/PB5 次要输入(开关/脉冲)处理
 *
 * 从 motor_control.c 拆分而来, 负责次要数字输入:
 *   - PIN_FUNC_STOP      : 停止按钮(按下强制PWM=0)
 *   - PIN_FUNC_HOME_START: 执行复位操作按钮(边沿触发StartHoming)
 *   - PIN_FUNC_START_FLAG: 启动标志位触发(边沿触发MODBUS_TriggerStartFlag)
 *   - PIN_FUNC_PULSE     : 脉冲输入(EXTI边沿累加到 pulse_accumulator)
 *   - PIN_FUNC_DIR       : 方向输入(供脉冲方向判定)
 *
 * 与电机控制模块的共享状态(由本模块产生, motor_control 消费):
 *   - pulse_accumulator : 脉冲累加值, MotorControl_Update 中原子读取并入目标位置
 *   - stop_override     : 停止覆盖标志, MotorControl_Update 中强制PWM=0
 *
 * 算法/功能与拆分前完全一致, 不改变任何行为.
 */
#ifndef __INPUT_H
#define __INPUT_H

#include "motor_control.h"

/* 初始化输入模块: 绑定电机控制实例指针并初始化PB4/PB5边沿检测状态.
 * 由 MotorControl_Init 在加载配置后(极性已知)调用 */
void Input_Init(MotorControl_t *mc);

/* 主循环中调用, 处理PB4/PB5输入(停止/复位/启动标志位按钮) */
void MotorControl_ProcessInputs(void);

/* 根据PB4/PB5功能配置EXTI外部中断, 仅脉冲模式启用 */
void MotorControl_UpdateIrqPriority(void);

/* EXTI中断中调用, pin=4或5, 累加脉冲到目标位置 */
void MotorControl_OnPulseExti(uint8_t pin);

/* 由本模块产生、供 motor_control 模块消费的共享状态 */
extern volatile int32_t pulse_accumulator;  /* 待消费的脉冲累加值 */
extern uint8_t stop_override;               /* 0=正常, 1=停止(PWM=0) */

#endif /* __INPUT_H */
