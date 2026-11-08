/**
 * @file encoder.h
 * @brief 编码器模块 - 64位位置合成与速度读取
 *
 * 从 motor_control.c 拆分而来, 仅包含编码器相关功能:
 *   - TIM3 16位CNT的100μs差分累加, 合成64位无歧义位置
 *   - 位置/速度读取(含方向取反)
 *   - 编码器复位
 *
 * 与电机控制模块的耦合点(通过绑定接口解耦, 避免循环依赖):
 *   - 方向标志: Encoder_BindDirection() 由 MotorControl_Init /
 *     MotorControl_SetEncoderDirection 在方向变更时同步
 *   - 滤波速度源: Encoder_BindVelocity() 绑定 motor_control.c 的 current_velocity
 *
 * 行为与拆分前完全一致, 不改变任何算法和功能.
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 绑定编码器方向反转标志 (0=正常, 非0=反转: 计数/速度取反)
 * 由 MotorControl_Init / MotorControl_SetEncoderDirection 调用 */
void Encoder_BindDirection(uint8_t reversed);

/* 绑定滤波速度源指针(指向 motor_control.c 的 current_velocity)
 * 由 MotorControl_Init 调用 */
void Encoder_BindVelocity(float *velocity_source);

/* TIM1 100μs中断调用: 差分累加 TIM3->CNT 合成64位位置 */
void Encoder_Synthesize(void);

/* 读取64位累计位置(含方向取反, 关中断原子读取) */
int64_t Encoder_GetCount(void);

/* 读取滤波后速度(含方向取反) */
float Encoder_GetSpeed(void);

/* 编码器清零(CNT/prev/total 原子清零, 关全局中断) */
void Encoder_Reset(void);

/* 设置编码器绝对计数(含方向取反, 关全局中断原子写入).
 * 用于复位完成后将原点设定为计算出的位置, 而非实际停留位置. */
void Encoder_SetCount(int64_t count);

/* TIM3溢出中断已弃用, 保留空函数避免链接错误 */
void Encoder_OverflowHandler(void);

#endif /* __ENCODER_H */
