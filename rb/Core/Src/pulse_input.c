/**
 * @file pulse_input.c
 * @brief 脉冲输入模块实现 - TIM2外部时钟模式1 + 脉冲/方向分离计数
 *
 * 架构说明:
 *   - TIM2配置为External Clock Mode 1 (SMCR: SMS=111, TS=101),
 *     TI1(PA15)上升沿触发CNT+1, 只计单边沿(适合脉冲+方向应用)
 *   - 方向由PB3电平决定, 在PulseInput_Synthesize中软件判断:
 *       PB3=高 → 正方向(pulse_total_count += delta)
 *       PB3=低 → 负方向(pulse_total_count -= delta)
 *   - 64位位置合成:
 *       delta = (int16_t)(cnt_curr - cnt_prev)  // 外部时钟模式CNT只向上计数
 *       if (PB3=高) pulse_total_count += delta;
 *       else         pulse_total_count -= delta;
 *
 * 为什么不用编码器模式(TIM_ENCODERMODE_TI1)?
 *   编码器模式是双边沿计数, 一个脉冲周期会计数两次:
 *     - TI1上升沿: TI2=高→CNT-1, TI2=低→CNT+1
 *     - TI1下降沿: TI2=高→CNT+1, TI2=低→CNT-1
 *   若PB3=高(方向正), 一个脉冲周期: 上升沿-1 + 下降沿+1 = 净变化0
 *   这会导致CNT在0和1之间震荡, 目标位置不累加.
 *   外部时钟模式1只在TI1上升沿计数一次, 完美解决此问题.
 *
 * 引脚切换:
 *   - 脉冲模式: PA15/PB3配置为输入+上拉, 启动TIM2外部时钟模式1
 *   - 其他模式: PA15/PB3配置为普通GPIO输入, 由input.c直接读IDR
 *
 * 64位原子读: 主循环读取pulse_total_count时关全局中断, 避免高/低32位撕裂.
 */
#include "pulse_input.h"
#include "stm32f1xx_hal.h"

/* htim2 由 main.c 中 MX_TIM2_Init 初始化, 此处extern声明引用
 * (与 motor_control.c 引用 htim3/htim4 的方式一致) */
extern TIM_HandleTypeDef htim2;

/* ========== 64位脉冲累计计数(在TIM1 100μs中断中合成) ==========
 * 外部时钟模式1下CNT只向上计数, 方向由PB3电平在Synthesize中决定:
 *   - TIM2的16位CNT在0~65535循环, 跨越65535→0时int16_t差分自动处理
 *   - 物理约束: 100μs内脉冲不可能走32768步(需327MHz脉冲频率),
 *     故int16_t差分绝不会歧义 */
static volatile int64_t pulse_total_count = 0;  /* 64位累计脉冲数(含方向) */
static uint16_t pulse_cnt_prev = 0;             /* 上次读到的CNT值(仅TIM1 ISR访问) */

/* 由 motor_control 模块绑定的方向反转标志 */
static uint8_t pulse_dir_reversed = 0;   /* 0=正常, 1=反转(计数取反) */

/* 绑定的电机控制实例(内部链接, 与input.c的motor_ctl互不影响) */
static MotorControl_t *pulse_mc = NULL;

/* TIM2是否已启动(防止重复Start/Stop) */
static uint8_t tim2_running = 0;

void PulseInput_Init(MotorControl_t *mc)
{
    pulse_mc = mc;
    pulse_dir_reversed = 0;
    pulse_total_count = 0;
    pulse_cnt_prev = 0;
    tim2_running = 0;
}

/* 配置PA15/PB3为TIM2输入(STM32F1定时器编码器/外部时钟模式都用普通输入即可)
 *
 * 关键: 必须用GPIO_MODE_INPUT而非GPIO_MODE_AF_INPUT!
 * 原因: STM32F1的GPIO_MODE_AF_INPUT会强制CNF=01(浮空输入), Pull设置被忽略.
 *       GPIO_MODE_INPUT的CNF=10(上拉/下拉), 配合GPIO_PULLUP才能让上拉生效.
 * 参考: MX生成的TIM3编码器MspInit也是用GPIO_MODE_INPUT(STM32F1标准做法). */
static void config_pins_af_input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PA15 = TIM2_CH1 (脉冲输入) - 普通输入+上拉
     * 注意: PA15默认是JTAG的JTDI, 使用前必须禁用JTAG */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB3 = TIM2_CH2 (方向输入) - 普通输入+上拉
     * 上拉确保无外部方向信号时PB3=高电平, 脉冲方向恒为正(pulse_total_count递增).
     * 注意: PB3默认是JTAG的JTDO, 使用前必须禁用JTAG */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* 配置PA15/PB3为普通GPIO输入 */
static void config_pins_gpio_input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PA15 - 普通输入, 上拉(适用于按键/限位/方向等) */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB3 - 普通输入, 上拉 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief 重新配置TIM2为外部时钟模式1
 *
 * 外部时钟模式1 vs 编码器模式:
 *   - 编码器模式(TIM_ENCODERMODE_TI1): TI1双边沿计数, 方向由TI2决定
 *     → 一个脉冲周期会计数两次(+1-1=0), 不适合"脉冲+方向"应用
 *   - 外部时钟模式1(SMCR SMS=111): TI1单边沿计数, CNT只向上递增
 *     → 一个脉冲周期只计数一次(+1), 方向由PB3电平软件判断
 *
 * SMCR寄存器配置:
 *   - SMS[2:0] = 111 (External Clock Mode 1)
 *   - TS[2:0]  = 101 (Filtered Timer Input 1, TI1FP1作为外部时钟源)
 *
 * CCMR1寄存器配置:
 *   - CC1S[1:0] = 01 (CC1通道配置为输入, IC1映射到TI1)
 *
 * CCER寄存器配置:
 *   - CC1P = 0 (非反转, 上升沿计数; =1则为下降沿计数)
 */
static void config_tim2_external_clock_mode(void)
{
    /* 先禁用TIM2 (CR1.CEN=0) 才能安全修改配置 */
    htim2.Instance->CR1 = 0;

    /* 清除所有配置, 然后设置外部时钟模式1 */
    htim2.Instance->SMCR = 0;  /* 先清零 */
    htim2.Instance->SMCR = (5 << 4) | 7;  /* TS=101(TI1FP1), SMS=111(External Clock Mode 1) */

    /* CC1配置为输入: CC1S=01, IC1映射到TI1 */
    htim2.Instance->CCMR1 = 0x0001;

    /* CC1P=0 (上升沿计数, 非反转) */
    htim2.Instance->CCER = 0;

    /* PSC=0, ARR=0xFFFF, 计数器模式=向上 */
    htim2.Instance->PSC = 0;
    htim2.Instance->ARR = 0xFFFF;

    /* 生成更新事件, 加载PSC/ARR到影子寄存器 */
    htim2.Instance->EGR = TIM_EGR_UG;

    /* 清零CNT */
    htim2.Instance->CNT = 0;

    /* 启动TIM2 (CR1.CEN=1) */
    htim2.Instance->CR1 = TIM_CR1_CEN;
}

void PulseInput_Reconfig(MotorControl_t *mc)
{
    if (!mc) return;

    uint8_t pulse_on_pin4 = (mc->pin4_func == PIN_FUNC_PULSE);
    uint8_t pulse_on_pin5 = (mc->pin5_func == PIN_FUNC_PULSE);

    /* 先停止TIM2(若在运行) */
    PulseInput_Stop();

    if (pulse_on_pin4 || pulse_on_pin5) {
        /* 脉冲模式: 配置PA15/PB3为输入+上拉, 启动TIM2外部时钟模式1 */

        /* 禁用JTAG, 保留SWD(PA13/PA14), 释放PA15/JTDI和PB3/JTDO
         * __HAL_AFIO_REMAP_SWJ_NOJTAG 会设置 AFIO_MAPR SWJ_CFG = 010
         * 注意: 必须在GPIO配置前禁用JTAG, 否则PA15/PB3无法被GPIO控制器接管 */
        __HAL_AFIO_REMAP_SWJ_NOJTAG();

        /* 配置PA15/PB3为输入+上拉 */
        config_pins_af_input();

        /* 方向反转标志: 若脉冲引脚为ACTIVE_LOW, 翻转计数方向
         * (脉冲极性低电平有效时, 方向判定取反, 等效于CNT方向取反) */
        if (pulse_on_pin4) {
            pulse_dir_reversed = (mc->pin4_polarity == POLARITY_ACTIVE_LOW) ? 1 : 0;
        } else {
            pulse_dir_reversed = (mc->pin5_polarity == POLARITY_ACTIVE_LOW) ? 1 : 0;
        }

        /* 清零CNT和64位计数 */
        pulse_cnt_prev = 0;
        pulse_total_count = 0;

        /* 配置并启动TIM2外部时钟模式1: TI1(PA15)上升沿计数 */
        config_tim2_external_clock_mode();
        tim2_running = 1;
    } else {
        /* 非脉冲模式: 配置为普通GPIO输入, 由input.c处理 */
        /* 同样需要禁用JTAG才能使用PA15/PB3 */
        __HAL_AFIO_REMAP_SWJ_NOJTAG();
        config_pins_gpio_input();
    }
}

void PulseInput_Synthesize(void)
{
    /* 仅在TIM2运行时才合成, 避免读取未启动的CNT(值无意义) */
    if (!tim2_running) return;

    /* 差分累加TIM2->CNT到64位
     * 外部时钟模式1下CNT只向上计数, delta始终为正(脉冲数)
     * 方向由PB3电平决定 */
    uint16_t cnt_curr = (uint16_t)TIM2->CNT;
    int16_t delta = (int16_t)(cnt_curr - pulse_cnt_prev);
    pulse_cnt_prev = cnt_curr;

    if (delta != 0) {
        /* 外部时钟模式1: CNT只向上计数, 方向由PB3电平决定
         * PB3=高 → 正方向(脉冲累加)
         * PB3=低 → 负方向(脉冲减少)
         * 这里读取PB3电平决定方向, 替代编码器模式的硬件方向判断 */
        uint8_t pb3_high = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) ? 1 : 0;
        if (pb3_high) {
            pulse_total_count += delta;  /* 正方向 */
        } else {
            pulse_total_count -= delta;  /* 负方向 */
        }
    }
}

int32_t PulseInput_ConsumeDelta(MotorControl_t *mc)
{
    (void)mc;  /* 当前未使用mc参数, 保留接口一致性 */

    /* 关全局中断保证64位原子读, 避免高/低32位撕裂
     * 关中断时间极短(几条指令), 对系统无影响 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int64_t total = pulse_total_count;
    pulse_total_count = 0;
    __set_PRIMASK(primask);

    /* 方向反转时取反 */
    if (pulse_dir_reversed) {
        total = -total;
    }

    /* 限制单次消费的脉冲数, 防止异常情况下target_position突变
     * (理论上100μs内脉冲数有限, 这里取保守上限65535) */
    if (total > 65535) total = 65535;
    if (total < -65535) total = -65535;

    return (int32_t)total;
}

void PulseInput_Start(void)
{
    if (tim2_running) return;

    /* 仅在脉冲模式时启动TIM2(由PulseInput_Reconfig已配置好输入) */
    uint8_t pulse_active = 0;
    if (pulse_mc) {
        pulse_active = (pulse_mc->pin4_func == PIN_FUNC_PULSE) ||
                       (pulse_mc->pin5_func == PIN_FUNC_PULSE);
    }
    if (!pulse_active) return;

    /* 清零CNT和64位计数 */
    pulse_cnt_prev = 0;
    pulse_total_count = 0;

    /* 配置并启动TIM2外部时钟模式1 */
    config_tim2_external_clock_mode();
    tim2_running = 1;
}

void PulseInput_Stop(void)
{
    if (!tim2_running) return;

    /* 停止TIM2 (CR1.CEN=0) */
    htim2.Instance->CR1 = 0;
    tim2_running = 0;
}

int64_t PulseInput_GetCount(void)
{
    /* 关全局中断保证64位原子读 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int64_t count = pulse_total_count;
    __set_PRIMASK(primask);

    if (pulse_dir_reversed) {
        count = -count;
    }
    return count;
}

void PulseInput_Reset(void)
{
    /* 关全局中断保证CNT/prev/total三者原子清零 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (tim2_running) {
        TIM2->CNT = 0;
    }
    pulse_cnt_prev = 0;
    pulse_total_count = 0;
    __set_PRIMASK(primask);
}
