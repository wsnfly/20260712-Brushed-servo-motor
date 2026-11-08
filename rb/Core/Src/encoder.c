/**
 * @file encoder.c
 * @brief 编码器模块实现 - 64位位置合成
 *
 * 从 motor_control.c 拆分而来. 算法/功能与拆分前完全一致:
 *   - TIM3->CNT 在 TIM1 100μs中断里做 int16_t 差分累加, 合成64位位置
 *   - 位置/速度读取时按 encoder_dir 决定是否取反
 *   - 复位时关全局中断保证 CNT/prev/total 三者原子清零
 *
 * 方向标志与速度源通过 Encoder_BindDirection/Encoder_BindVelocity 绑定,
 * 由 motor_control 模块在初始化及方向变更时同步, 避免本模块反向依赖 motor_control.h.
 */
#include "encoder.h"

/* ========== 64位编码器累计计数(在TIM1 100μs中断中合成) ==========
 * 设计原理: TIM3的16位CNT在0~65535之间循环, 跨越65535↔0时方向不定.
 * 旧方案依赖TIM3溢出中断修正wrap, 但中断响应有几条指令的窗口,
 * 在该窗口内主程序读CNT+wrap会合成出错误值(如1*65536+65535=131071).
 *
 * 新方案: 关闭TIM3溢出中断, 在TIM1 100μs中断里主动读CNT做差分累加.
 *   delta = (int16_t)(cnt_curr - cnt_prev)  // 补码自动处理65536模回绕
 *   total_count += delta
 * 物理约束: 100μs内编码器不可能走32768步(需327MHz编码器频率),
 * 因此int16_t差分绝不会歧义, 彻底消除"中断未执行"窗口的累计误差. */
static volatile int64_t encoder_total_count = 0;  /* 64位累计位置(不含方向取反) */
static uint16_t encoder_cnt_prev = 0;             /* 上次读到的CNT值(仅TIM1 ISR访问) */

/* 由 motor_control 模块绑定的方向标志与速度源 */
static uint8_t encoder_dir_reversed = 0;   /* 0=正常, 1=反转(计数/速度取反) */
static float *encoder_p_velocity = NULL;   /* 指向 motor_control.c 的 current_velocity */

void Encoder_BindDirection(uint8_t reversed)
{
    encoder_dir_reversed = reversed ? 1 : 0;
}

void Encoder_BindVelocity(float *velocity_source)
{
    encoder_p_velocity = velocity_source;
}

/* TIM3溢出中断已弃用: 保留空函数避免stm32f1xx_it.c中的调用链接错误.
 * 64位位置合成改为在TIM1 100μs中断里调用Encoder_Synthesize()实现. */
void Encoder_OverflowHandler(void)
{
    /* 空函数 - TIM3溢出中断已关闭, 不会进入此处 */
}

/* 编码器位置合成: 在TIM1 100μs中断中调用, 差分累加CNT.
 * 利用int16_t补码自动处理65536模回绕:
 *   正向溢出 65535→0:   delta = (int16_t)(0 - 65535) = (int16_t)(-65535) = +1
 *   反向溢出 0→65535:   delta = (int16_t)(65535 - 0) = (int16_t)(65535) = -1
 *   正常递增 100→200:   delta = +100
 *   正常递减 200→100:   delta = -100
 * 100μs内编码器不可能走32768步(物理约束), 故int16_t差分无歧义. */
void Encoder_Synthesize(void)
{
    uint16_t cnt_curr = (uint16_t)TIM3->CNT;
    int16_t delta = (int16_t)(cnt_curr - encoder_cnt_prev);
    encoder_total_count += delta;
    encoder_cnt_prev = cnt_curr;
}

int64_t Encoder_GetCount(void)
{
    /* encoder_total_count在TIM1 100μs中断里更新(64位).
     * 主循环读取时必须关全局中断保证64位读原子性,
     * 避免读到"高32位已更新/低32位未更新"的撕裂值.
     * 关中断时间极短(几条指令), 对系统无影响.
     * 在TIM1 ISR中调用时__get_PRIMASK()=1, __disable_irq()无效但安全. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int64_t count = encoder_total_count;
    __set_PRIMASK(primask);

    /* 编码器方向反转时计数取反 */
    if (encoder_dir_reversed) {
        count = -count;
    }
    return count;
}

float Encoder_GetSpeed(void)
{
    /* 未绑定速度源时返回0(等价于拆分前 current_velocity 初值0) */
    float spd = encoder_p_velocity ? *encoder_p_velocity : 0.0f;
    /* 编码器方向反转时速度取反 */
    if (encoder_dir_reversed) {
        spd = -spd;
    }
    return spd;
}

void Encoder_Reset(void)
{
    /* 关全局中断保证CNT/prev/total三者的同步原子性.
     * Encoder_Reset可能被以下场景调用:
     *   - 主循环(如MotorControl_SetOrigin已关TIM1中断, 这里再关全局更稳妥)
     *   - TIM1 ISR内(如MotorControl_FinishHoming), 此时已禁中断, 重复禁用安全 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TIM3->CNT = 0;
    encoder_cnt_prev = 0;
    encoder_total_count = 0;
    __set_PRIMASK(primask);
}

void Encoder_SetCount(int64_t count)
{
    /* 将编码器绝对计数设为指定值(逻辑计数, 含方向取反, 与Encoder_GetCount对称).
     * 存储原始值: 方向反转时取反存储, 读取时再取反得到相同逻辑计数.
     * 与Reset不同: 不强制TIM3->CNT清零, 而是同步cnt_prev为当前CNT,
     * 保证后续Synthesize的差分delta从当前CNT继续累加, 不产生跳变.
     * 用于复位完成后将原点设定为计算出的位置. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (encoder_dir_reversed) {
        count = -count;
    }
    encoder_total_count = count;
    encoder_cnt_prev = (uint16_t)TIM3->CNT;
    __set_PRIMASK(primask);
}
