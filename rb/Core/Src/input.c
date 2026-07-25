/**
 * @file input.c
 * @brief 输入模块实现 - PB4/PB5 次要输入处理
 *
 * 从 motor_control.c 拆分而来. 所有处理逻辑(去抖/冷却/边沿检测/EXTI配置/脉冲累加)
 * 与拆分前逐字一致, 不改变任何算法和功能.
 *
 * 说明: 本文件内 static MotorControl_t *motor_ctl 为输入模块自有的实例指针
 *       (内部链接), 由 Input_Init 设置, 与 motor_control.c 中同名 static 变量
 *       互不影响(C语言内部链接规则). 二者指向同一 MotorControl_t 实例.
 */
#include "input.h"

/* MODBUS 模块提供的启动标志位触发接口(同 motor_control.c 中的前向声明方式) */
extern void MODBUS_TriggerStartFlag(void);

/* 输入模块自有的电机控制实例指针(内部链接) */
static MotorControl_t *motor_ctl = NULL;

/* PB4/PB5 输入状态跟踪(拆分前位于 motor_control.c, 仅本模块写入) */
static uint8_t prev_pin4_state = 0;
static uint8_t prev_pin5_state = 0;

/* "执行复位操作"按钮(PIN_FUNC_HOME_START)边沿检测与去抖
 * 按下瞬间触发一次 MotorControl_StartHoming, 期间不重复触发
 * 防抖冷却: 上次触发后 HOME_START_COOLDOWN_MS 内的边沿被忽略 */
#define HOME_START_COOLDOWN_MS  300   /* 触发冷却时间, 兼作防抖 */
static uint8_t home_start_btn4_prev = 0;   /* PB4上次稳定电平(已按极性归一化) */
static uint8_t home_start_btn5_prev = 0;   /* PB5上次稳定电平 */
static uint32_t home_start_last_trigger_tick = 0;  /* 上次触发时刻(HAL_GetTick ms) */

/* "启动标志位触发"(PIN_FUNC_START_FLAG)边沿检测与去抖
 * 引脚有效边沿(上升沿/下降沿由极性决定)触发一次启动标志位,
 * 用于多电机同步启动: 上位机批量下发目标后, 硬件信号同步触发各电机启动.
 * 冷却复用 HOME_START_COOLDOWN_MS 防抖, 与HOME_START共享冷却计数器避免抖动叠加 */
static uint8_t startflag_btn4_prev = 0;   /* PB4上次稳定电平(已按极性归一化) */
static uint8_t startflag_btn5_prev = 0;   /* PB5上次稳定电平 */
static uint32_t startflag_last_trigger_tick = 0;  /* 上次触发时刻(HAL_GetTick ms) */

/* 由本模块产生、供 motor_control 消费的共享状态(在 input.h 中 extern 声明) */
volatile int32_t pulse_accumulator = 0;   /* 待消费的脉冲累加值 */
uint8_t stop_override = 0;                /* 0=正常, 1=停止(PWM=0) */

void Input_Init(MotorControl_t *mc)
{
    motor_ctl = mc;
    /* 初始化边沿检测状态（加载配置后根据极性初始化，避免首次触发虚假脉冲） */
    prev_pin4_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET ? 1 : 0;
    prev_pin5_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET ? 1 : 0;
    if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) prev_pin4_state = !prev_pin4_state;
    if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) prev_pin5_state = !prev_pin5_state;
}

/* 主循环中处理PB4/PB5输入（备份检查，定时器中断已处理脉冲/限位）
 *
 * 当前职责:
 *   - 处理配置为 PIN_FUNC_HOME_START("执行复位操作") 的引脚:
 *     按下瞬间(上升沿)触发一次 MotorControl_StartHoming。
 *     受 home_mode 配置约束: home_mode=HOME_MODE_OFF 时 StartHoming 内部直接返回。
 *     复位进行中(IsHoming=1)时不重复触发, 冷却 HOME_START_COOLDOWN_MS 防抖。
 *   - 处理配置为 PIN_FUNC_START_FLAG("启动标志位触发") 的引脚:
 *     引脚有效边沿触发一次 MODBUS_TriggerStartFlag, 等同于上位机写 REG_START_MODE=1。
 *     仅在 start_mode=1(标志位启动) 时有效, 用于多电机同步启动。
 *     复位进行中(IsHoming=1)时不触发, 冷却 HOME_START_COOLDOWN_MS 防抖。 */
void MotorControl_ProcessInputs(void)
{
    if (!motor_ctl) return;

    /* ========== 停止功能处理（PIN_FUNC_STOP）==========
     * 按钮按下时stop_override=1强制PWM输出为0,
     * 松开后stop_override=0恢复正常运行。
     * 不依赖Homing状态, 任何时刻都生效。 */
    {
        uint8_t stop_active = 0;
        if (motor_ctl->pin4_func == PIN_FUNC_STOP) {
            uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
            if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
            if (cur) stop_active = 1;
        }
        if (motor_ctl->pin5_func == PIN_FUNC_STOP) {
            uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
            if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
            if (cur) stop_active = 1;
        }
        stop_override = stop_active;
    }

    /* 仅在非复位状态处理按钮, 避免与正在进行的复位流程互相干扰 */
    if (MotorControl_IsHoming()) {
        /* 复位期间持续刷新"上次电平", 防止复位结束时误把松开动作当成新按下 */
        if (motor_ctl->pin4_func == PIN_FUNC_HOME_START) {
            uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
            if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
            home_start_btn4_prev = cur;
        }
        if (motor_ctl->pin5_func == PIN_FUNC_HOME_START) {
            uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
            if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
            home_start_btn5_prev = cur;
        }
        if (motor_ctl->pin4_func == PIN_FUNC_START_FLAG) {
            uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
            if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
            startflag_btn4_prev = cur;
        }
        if (motor_ctl->pin5_func == PIN_FUNC_START_FLAG) {
            uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
            if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
            startflag_btn5_prev = cur;
        }
        return;
    }

    uint32_t now = HAL_GetTick();

    /* PB4 配置为"执行复位操作"按钮 */
    if (motor_ctl->pin4_func == PIN_FUNC_HOME_START) {
        uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
        if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
        /* 上升沿 + 冷却时间已过 → 触发一次复位 */
        if (cur && !home_start_btn4_prev &&
            (now - home_start_last_trigger_tick) >= HOME_START_COOLDOWN_MS) {
            MotorControl_StartHoming(motor_ctl);
            home_start_last_trigger_tick = now;
        }
        home_start_btn4_prev = cur;
    } else {
        home_start_btn4_prev = 0;
    }

    /* PB5 配置为"执行复位操作"按钮 */
    if (motor_ctl->pin5_func == PIN_FUNC_HOME_START) {
        uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
        if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
        if (cur && !home_start_btn5_prev &&
            (now - home_start_last_trigger_tick) >= HOME_START_COOLDOWN_MS) {
            MotorControl_StartHoming(motor_ctl);
            home_start_last_trigger_tick = now;
        }
        home_start_btn5_prev = cur;
    } else {
        home_start_btn5_prev = 0;
    }

    /* ========== 启动标志位触发处理（PIN_FUNC_START_FLAG）==========
     * 引脚有效边沿(上升沿)触发一次 MODBUS_TriggerStartFlag,
     * 等同于上位机写 REG_START_MODE=1: 应用缓存的模式/目标位置/目标速度。
     * 仅在 start_mode=1 时由 MODBUS_TriggerStartFlag 内部判定生效,
     * start_mode=0(直接启动)时触发无动作(目标已在写入时即时生效)。
     * 用途: 多电机同步启动——上位机批量下发目标后, 硬件信号同步触发各电机。 */

    /* PB4 配置为"启动标志位触发" */
    if (motor_ctl->pin4_func == PIN_FUNC_START_FLAG) {
        uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
        if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
        if (cur && !startflag_btn4_prev &&
            (now - startflag_last_trigger_tick) >= HOME_START_COOLDOWN_MS) {
            MODBUS_TriggerStartFlag();
            startflag_last_trigger_tick = now;
        }
        startflag_btn4_prev = cur;
    } else {
        startflag_btn4_prev = 0;
    }

    /* PB5 配置为"启动标志位触发" */
    if (motor_ctl->pin5_func == PIN_FUNC_START_FLAG) {
        uint8_t cur = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
        if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) cur = !cur;
        if (cur && !startflag_btn5_prev &&
            (now - startflag_last_trigger_tick) >= HOME_START_COOLDOWN_MS) {
            MODBUS_TriggerStartFlag();
            startflag_last_trigger_tick = now;
        }
        startflag_btn5_prev = cur;
    } else {
        startflag_btn5_prev = 0;
    }
}

/* 根据PB4/PB5功能配置EXTI外部中断
 * 仅当引脚功能为 PIN_FUNC_PULSE 时启用EXTI边沿触发, 极性决定触发沿:
 *   POLARITY_ACTIVE_HIGH -> 上升沿触发
 *   POLARITY_ACTIVE_LOW  -> 下降沿触发
 * 非脉冲功能: 关闭EXTI并清挂起位, 防止残留触发
 *
 * STM32F1 EXTI线映射:
 *   EXTI4 通过 AFIO_EXTICR2 选择源引脚(0=PA4, 1=PB4, 2=PC4...)
 *   EXTI5 通过 AFIO_EXTICR2 选择源引脚(0=PA5, 1=PB5, 2=PC5...)
 * GPIO_MODE_INPUT 模式下HAL不配置 AFIO_EXTICR, 这里必须手动配置
 *
 * 中断优先级(固定, 不再动态切换):
 *   TIM1 PID=1 (main.c启动时设置), EXTI脉冲=1 (同级, 不互相抢占),
 *   USART1=3. EXTI有PR硬件挂起位锁存, 即便被TIM1延迟处理也不会丢失脉冲边沿. */
void MotorControl_UpdateIrqPriority(void)
{
    if (!motor_ctl) return;

    uint8_t pulse_on_pin4 = (motor_ctl->pin4_func == PIN_FUNC_PULSE);
    uint8_t pulse_on_pin5 = (motor_ctl->pin5_func == PIN_FUNC_PULSE);

    /* 配置EXTI4/EXTI5源为PB4/PB5 (AFIO_EXTICR2: bits[3:0]=EXTI4源, bits[7:4]=EXTI5源)
     * PB port source = 0x01, 写之前先屏蔽对应EXTI线避免配置期间误触发 */
    EXTI->IMR &= ~(EXTI_IMR_IM4 | EXTI_IMR_IM5);
    AFIO->EXTICR[1] = (AFIO->EXTICR[1] & ~0x00FFu) | 0x0011u;  /* EXTI4=PB4, EXTI5=PB5 */

    /* 配置PB4 EXTI线 (Line 4) */
    {
        EXTI->EMR &= ~EXTI_EMR_EM4;
        EXTI->RTSR &= ~EXTI_RTSR_TR4;   /* 清上升沿 */
        EXTI->FTSR &= ~EXTI_FTSR_TR4;   /* 清下降沿 */
        EXTI->PR = EXTI_PR_PR4;         /* 清挂起位, 写1清0 */

        if (pulse_on_pin4) {
            if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) {
                EXTI->FTSR |= EXTI_FTSR_TR4;  /* 低电平有效: 下降沿触发 */
            } else {
                EXTI->RTSR |= EXTI_RTSR_TR4;  /* 高电平有效: 上升沿触发 */
            }
            EXTI->IMR |= EXTI_IMR_IM4;       /* 取消屏蔽, 使能中断 */
        }
    }

    /* 配置PB5 EXTI线 (Line 5, 与Line 5~9共享 EXTI9_5_IRQHandler) */
    {
        EXTI->EMR &= ~EXTI_EMR_EM5;
        EXTI->RTSR &= ~EXTI_RTSR_TR5;
        EXTI->FTSR &= ~EXTI_FTSR_TR5;
        EXTI->PR = EXTI_PR_PR5;

        if (pulse_on_pin5) {
            if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) {
                EXTI->FTSR |= EXTI_FTSR_TR5;
            } else {
                EXTI->RTSR |= EXTI_RTSR_TR5;
            }
            EXTI->IMR |= EXTI_IMR_IM5;
        }
    }

    /* NVIC: EXTI优先级固定为1, 仅按需使能/失能NVIC (不再切换TIM1优先级)
     * TIM1_UP_IRQn优先级在main.c启动时一次性设为1 */
    HAL_NVIC_SetPriority(EXTI4_IRQn,   1, 0);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    if (pulse_on_pin4 || pulse_on_pin5) {
        HAL_NVIC_EnableIRQ(EXTI4_IRQn);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    } else {
        HAL_NVIC_DisableIRQ(EXTI4_IRQn);
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    }

    /* 同步内部边沿检测状态(归一化后电平), 避免切换瞬间误触发 */
    uint8_t p4 = (GPIOB->IDR & GPIO_PIN_4) ? 1 : 0;
    uint8_t p5 = (GPIOB->IDR & GPIO_PIN_5) ? 1 : 0;
    if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) p4 = !p4;
    if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) p5 = !p5;
    prev_pin4_state = p4;
    prev_pin5_state = p5;
}

/* EXTI中断中调用: 脉冲边沿处理 (pin=4或5)
 * 直接读GPIO IDR寄存器(2 cycles), 累加pulse_accumulator(atomic int32)
 * 全函数约30~50 cycles, 含硬件压栈出栈约580ns
 * 方向引脚在同一时刻读取, 比TIM2轮询更精准 */
void MotorControl_OnPulseExti(uint8_t pin)
{
    if (!motor_ctl) return;

    /* 当前直接读IDR寄存器, 避免HAL_GPIO_ReadPin的额外开销 */
    uint32_t idr = GPIOB->IDR;
    uint8_t cur_pin4 = (idr & GPIO_PIN_4) ? 1 : 0;
    uint8_t cur_pin5 = (idr & GPIO_PIN_5) ? 1 : 0;

    if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) cur_pin4 = !cur_pin4;
    if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) cur_pin5 = !cur_pin5;

    /* 查找方向引脚: PB4/PB5中配置为DIR的那个 */
    uint8_t dir_pin_state = 0;
    if (motor_ctl->pin4_func == PIN_FUNC_DIR) {
        dir_pin_state = cur_pin4;
    } else if (motor_ctl->pin5_func == PIN_FUNC_DIR) {
        dir_pin_state = cur_pin5;
    }

    if (pin == 4) {
        /* PB4脉冲: 根据方向脚累加+1或-1 */
        if (motor_ctl->pin4_func == PIN_FUNC_PULSE) {
            pulse_accumulator += dir_pin_state ? 1 : -1;
        }
        prev_pin4_state = cur_pin4;
    } else if (pin == 5) {
        if (motor_ctl->pin5_func == PIN_FUNC_PULSE) {
            pulse_accumulator += dir_pin_state ? 1 : -1;
        }
        prev_pin5_state = cur_pin5;
    }
}
