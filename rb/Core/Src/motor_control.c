/*
 * motor_control.c - 直流电机伺服控制器实现
 *
 * ============ 硬件连接 ============
 *
 * TIM4 (PWM - 电机驱动)
 *   CH1 (PB6) - H桥正向PWM (0~max_output)
 *   CH2 (PB7) - 使能1 (始终100%)
 *   CH3 (PB8) - H桥反向PWM (0~max_output)
 *   CH4 (PB9) - 使能2 (始终100%)
 *   时钟: APB1=8MHz, Prescaler=7 -> 1MHz
 *   Period=1000 -> PWM频率=1KHz
 *   最大输出900(90%), 留10%给自举电容充电
 *
 * TIM3 (编码器)
 *   CH1 (PA6) - 编码器A相
 *   CH2 (PA7) - 编码器B相
 *   模式: Encoder Mode TI1+TI2 (4倍频)
 *   更新中断: 溢出/下溢时扩展64位计数
 *
 * TIM1 (100us中断, 内部计数驱动PID)
 *   PSC=63, ARR=99 -> ~89us中断
 *   MotorControl_TimerTick100us() 每50次调用(约4.5ms)执行一次PID
 *
 * ============ 中断处理 ============
 *
 * 以下中断处理函数由本库提供:
 *   MotorControl_TimerTick100us - 由TIM1 100us中断调用
 *     内部每50次触发一次 MotorControl_Update()
 *
 * 以下中断处理函数需要用户在 stm32f1xx_it.c 中实现:
 *   TIM1_UP_IRQHandler -> 调用 MotorControl_TimerTick100us()
 *   TIM3_IRQHandler -> 调用 Encoder_OverflowHandler()
 *     void TIM3_IRQHandler(void) {
 *         if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
 *             __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
 *             Encoder_OverflowHandler();
 *         }
 *         HAL_TIM_IRQHandler(&htim3);
 *     }
 *
 * 调用顺序:
 *   main() -> MotorControl_Init() -> Motor_Start()
 *
 *   TIM1每~89us触发 -> TIM1_UP_IRQHandler
 *                    -> MotorControl_TimerTick100us()
 *                    -> 每50次: MotorControl_Update() -> Motor_SetPWM()
 *
 *   TIM3编码器溢出 -> TIM3_IRQHandler
 *                   -> Encoder_OverflowHandler()
 *                   -> 更新encoder_wrap
 */

#include "motor_control.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

extern void MODBUS_SyncTargetPosition(int64_t position);

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Flash存储配置 - 使用最后一页(1KB)保存断电不丢失的参数 */
#define FLASH_CONFIG_ADDR       0x0801FC00  /* 最后一页起始地址 */
#define FLASH_CONFIG_MAGIC      0x4D434E46  /* 魔数 "MCNF" */
#define FLASH_TARGET_MAGIC      0x54504754  /* 目标触发位置有效标记 "TPGT" */

/* Flash中存储的配置结构（全部用uint32_t，方便Flash按字编程） */
typedef struct {
    uint32_t magic;
    uint32_t pos_kp;
    uint32_t pos_ki;
    uint32_t pos_kd;
    uint32_t spd_kp;
    uint32_t spd_ki;
    uint32_t spd_kd;
    uint32_t dead_zone;
    uint32_t max_output;
    uint32_t direction;
    uint32_t encoder_dir;
    uint32_t start_mode;
    uint32_t modbus_addr;
    uint32_t modbus_baud;
    uint32_t home_mode;
    uint32_t home_dir;
    uint32_t home_current;
    uint32_t home_speed;
    uint32_t home_max_distance;
    uint32_t home_back_distance;
    uint32_t home_auto_start;
    uint32_t pin4_func;
    uint32_t pin4_polarity;
    uint32_t pin4_limit_dir;
    uint32_t pin5_func;
    uint32_t pin5_polarity;
    uint32_t pin5_limit_dir;
    uint32_t max_run_speed;
    uint32_t target_pos_magic;
    uint32_t pin4_target_pos_h;
    uint32_t pin4_target_pos_l;
    uint32_t pin5_target_pos_h;
    uint32_t pin5_target_pos_l;
    uint32_t home_precision_speed;
    uint32_t home_precision_cycles;
    uint32_t pin4_target_speed;
    uint32_t pin5_target_speed;
    uint32_t mode;
} FlashConfig_t;

typedef enum {
    HOMING_IDLE = 0,
    HOMING_SEARCH,
    HOMING_BACK,
    HOMING_SWITCH_SEARCH,              /* 开关复位：向负方向搜索开关 */
    HOMING_SWITCH_RELEASE,             /* 开关复位：向正方向离开开关 */
    HOMING_SWITCH_PRECISION_APPROACH,  /* 开关复位：低速接近开关 */
    HOMING_SWITCH_PRECISION_RELEASE,   /* 开关复位：低速离开开关 */
    HOMING_SWITCH_BACK                 /* 开关复位：退回偏置距离后设为零点 */
} HomingState_t;

#define HOMING_STALL_TICKS       30U   /* 30 * 5ms = 150ms */
#define HOMING_STALL_GRACE_TICKS 40U   /* 启动后200ms内不判堵转 */

/* float <-> uint32_t 转换 */
static uint32_t float_to_u32(float f) {
    uint32_t u;
    memcpy(&u, &f, 4);
    return u;
}
static float u32_to_float(uint32_t u) {
    float f;
    memcpy(&f, &u, 4);
    return f;
}

/* 从Flash加载配置，无有效数据时保留传入的默认值 */
void MotorControl_LoadConfig(MotorControl_t *mc)
{
    FlashConfig_t *cfg = (FlashConfig_t *)FLASH_CONFIG_ADDR;
    if (cfg->magic == FLASH_CONFIG_MAGIC) {
        /* 加载PID参数并检查合理性（NaN或0视为无效，保留默认值） */
        float f;
        f = u32_to_float(cfg->pos_kp);
        if (f == f && f > 0.0f) mc->pos_Kp = f;  /* f==f 排除NaN */
        f = u32_to_float(cfg->pos_ki);
        if (f == f && f >= 0.0f) mc->pos_Ki = f;
        f = u32_to_float(cfg->pos_kd);
        if (f == f && f >= 0.0f) mc->pos_Kd = f;
        f = u32_to_float(cfg->spd_kp);
        if (f == f && f > 0.0f) mc->spd_Kp = f;
        f = u32_to_float(cfg->spd_ki);
        if (f == f && f >= 0.0f) mc->spd_Ki = f;
        f = u32_to_float(cfg->spd_kd);
        if (f == f && f >= 0.0f) mc->spd_Kd = f;

        /* max_output 必须在合理范围 [10, 1000]，否则保留默认值 */
        int16_t mo = (int16_t)cfg->max_output;
        if (mo >= 10 && mo <= 1000) {
            mc->max_output = mo;
        }
        /* dead_zone 范围检查 [0, 1000] */
        int16_t dz = (int16_t)cfg->dead_zone;
        if (dz >= 0 && dz <= 1000) {
            mc->dead_zone = dz;
        }

        mc->direction = (cfg->direction != 0) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
        mc->encoder_dir = (cfg->encoder_dir != 0) ? ENCODER_DIR_REVERSED : ENCODER_DIR_NORMAL;
        mc->start_mode = (uint8_t)(cfg->start_mode & 0x01);
        if (cfg->modbus_addr >= 1 && cfg->modbus_addr <= 247) {
            mc->modbus_addr = (uint8_t)cfg->modbus_addr;
        }
        if (cfg->modbus_baud == 9600 || cfg->modbus_baud == 19200 ||
            cfg->modbus_baud == 38400 || cfg->modbus_baud == 57600 ||
            cfg->modbus_baud == 115200) {
            mc->modbus_baud = cfg->modbus_baud;
        }
        if (cfg->home_mode <= 2) {
            mc->home_mode = (uint8_t)cfg->home_mode;
        }
        if (cfg->home_dir <= 1) {
            mc->home_dir = (cfg->home_dir != 0) ? 1 : 0;
        }
        if (cfg->home_current >= 10 && cfg->home_current <= 1000) {
            mc->home_current = (int16_t)cfg->home_current;
        }
        if (cfg->home_speed > 0 && cfg->home_speed <= 1000000) {
            mc->home_speed = (int32_t)cfg->home_speed;
        }
        if (cfg->home_max_distance > 0 && cfg->home_max_distance <= 1000000000) {
            mc->home_max_distance = (int32_t)cfg->home_max_distance;
        }
        if (cfg->home_back_distance >= -1000000000 && cfg->home_back_distance <= 1000000000) {
            mc->home_back_distance = (int32_t)cfg->home_back_distance;
        }
        mc->home_auto_start = (cfg->home_auto_start != 0) ? 1 : 0;
        if (cfg->home_precision_speed > 0 && cfg->home_precision_speed <= 1000000) {
            mc->home_precision_speed = (int32_t)cfg->home_precision_speed;
        }
        if (cfg->home_precision_cycles >= 1 && cfg->home_precision_cycles <= 20) {
            mc->home_precision_cycles = (uint8_t)cfg->home_precision_cycles;
        }
        mc->pin4_target_speed = (int32_t)cfg->pin4_target_speed;
        mc->pin5_target_speed = (int32_t)cfg->pin5_target_speed;
        if (cfg->mode <= EXTERNAL_TARGET_SPEED_MODE) {
            mc->mode = (MotorMode_t)cfg->mode;
        }
        if (cfg->pin4_func <= PIN_FUNC_TARGET) {
            mc->pin4_func = (uint8_t)cfg->pin4_func;
        }
        if (cfg->pin4_polarity <= 1) {
            mc->pin4_polarity = (uint8_t)cfg->pin4_polarity;
        }
        if (cfg->pin4_limit_dir <= 1) {
            mc->pin4_limit_dir = (uint8_t)cfg->pin4_limit_dir;
        }
        if (cfg->pin5_func <= PIN_FUNC_TARGET) {
            mc->pin5_func = (uint8_t)cfg->pin5_func;
        }
        if (cfg->pin5_polarity <= 1) {
            mc->pin5_polarity = (uint8_t)cfg->pin5_polarity;
        }
        if (cfg->pin5_limit_dir <= 1) {
            mc->pin5_limit_dir = (uint8_t)cfg->pin5_limit_dir;
        }
        if (cfg->max_run_speed <= 1000000) {
            mc->max_run_speed = (int32_t)cfg->max_run_speed;
        }
        if (cfg->target_pos_magic == FLASH_TARGET_MAGIC) {
            mc->pin4_target_pos = (int64_t)(((uint64_t)cfg->pin4_target_pos_h << 32) | cfg->pin4_target_pos_l);
            mc->pin5_target_pos = (int64_t)(((uint64_t)cfg->pin5_target_pos_h << 32) | cfg->pin5_target_pos_l);
        }
    }
    /* magic不匹配时保留Init中设置的默认值，不覆盖 */
}

/* 将配置写入Flash */
void MotorControl_SaveConfig(MotorControl_t *mc)
{
    HAL_FLASH_Unlock();

    /* 擦除最后一页 */
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_CONFIG_ADDR;
    erase.NbPages = 1;
    HAL_FLASHEx_Erase(&erase, &page_error);

    /* 按顺序写入所有参数 */
    uint32_t addr = FLASH_CONFIG_ADDR;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr,        FLASH_CONFIG_MAGIC);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4,  float_to_u32(mc->pos_Kp));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 8,  float_to_u32(mc->pos_Ki));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 12, float_to_u32(mc->pos_Kd));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 16, float_to_u32(mc->spd_Kp));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 20, float_to_u32(mc->spd_Ki));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 24, float_to_u32(mc->spd_Kd));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 28, (uint32_t)mc->dead_zone);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 32, (uint32_t)mc->max_output);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 36, (uint32_t)mc->direction);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 40, (uint32_t)mc->encoder_dir);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 44, (uint32_t)mc->start_mode);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 48, (uint32_t)mc->modbus_addr);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 52, mc->modbus_baud);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 56, (uint32_t)mc->home_mode);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 60, (uint32_t)mc->home_dir);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 64, (uint32_t)mc->home_current);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 68, (uint32_t)mc->home_speed);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 72, (uint32_t)mc->home_max_distance);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 76, (uint32_t)mc->home_back_distance);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 80, (uint32_t)mc->home_auto_start);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 84, (uint32_t)mc->pin4_func);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 88, (uint32_t)mc->pin4_polarity);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 92, (uint32_t)mc->pin4_limit_dir);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 96, (uint32_t)mc->pin5_func);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 100, (uint32_t)mc->pin5_polarity);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 104, (uint32_t)mc->pin5_limit_dir);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 108, (uint32_t)mc->max_run_speed);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 112, FLASH_TARGET_MAGIC);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 116, (uint32_t)((uint64_t)mc->pin4_target_pos >> 32));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 120, (uint32_t)((uint64_t)mc->pin4_target_pos & 0xFFFFFFFFULL));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 124, (uint32_t)((uint64_t)mc->pin5_target_pos >> 32));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 128, (uint32_t)((uint64_t)mc->pin5_target_pos & 0xFFFFFFFFULL));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 132, (uint32_t)mc->home_precision_speed);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 136, (uint32_t)mc->home_precision_cycles);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 140, (uint32_t)mc->pin4_target_speed);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 144, (uint32_t)mc->pin5_target_speed);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 148, (uint32_t)mc->mode);

    HAL_FLASH_Lock();
}

static uint8_t motor_running = 0;
static float current_velocity = 0.0f;
static float ramped_target_speed = 0.0f;
static volatile int32_t encoder_wrap = 0;
static int64_t last_valid_count = 0;  /* 上一次合成的有效计数值(不含方向取反), 用于竞争跳过时返回 */
static MotorControl_t *motor_ctl = NULL;
static volatile int16_t current_pwm_output = 0;  /* 当前PWM输出(-1000~+1000) */
static HomingState_t homing_state = HOMING_IDLE;
static uint8_t homing_failed = 0;
static uint32_t homing_ticks = 0;
static uint32_t homing_stall_ticks = 0;
static int64_t homing_start_position = 0;
static MotorMode_t mode_before_homing = POSITION_MODE;
static float speed_limit_integral = 0.0f;
static float speed_limit_prev_velocity = 0.0f;

#define SPEED_RAMP_ACCEL 10000.0f  /* 速度模式目标变化斜坡: 脉冲/秒^2 */

/* 开关精确复位专用变量 */
static int64_t switch_edge_pos = 0;       /* 开关边缘触发位置 */
static uint8_t precision_cycle_idx = 0;   /* 当前精确检测循环次数 */
static int64_t switch_edge_sum = 0;       /* 边缘位置累加（用于求平均） */

/* PB4/PB5 输入状态跟踪 */
static uint8_t prev_pin4_state = 0;
static uint8_t prev_pin5_state = 0;
static volatile int32_t pulse_accumulator = 0;
static uint8_t target_level_state = 0;

static int64_t abs64(int64_t v)
{
    return (v < 0) ? -v : v;
}

static float clampf(float v, float min_v, float max_v)
{
    if (v > max_v) return max_v;
    if (v < min_v) return min_v;
    return v;
}

static float MotorControl_UpdateSpeedRamp(float target, float dt)
{
    float max_step = SPEED_RAMP_ACCEL * dt;
    float delta = target - ramped_target_speed;

    if (delta > max_step) {
        ramped_target_speed += max_step;
    } else if (delta < -max_step) {
        ramped_target_speed -= max_step;
    } else {
        ramped_target_speed = target;
    }

    return ramped_target_speed;
}

/* 读取配置为HOME功能的引脚状态（有效=1，无效=0） */
static uint8_t ReadHomeSwitch(MotorControl_t *mc)
{
    uint8_t pin4 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t pin5 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
    if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) pin4 = !pin4;
    if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) pin5 = !pin5;
    if (mc->pin4_func == PIN_FUNC_HOME && pin4) return 1;
    if (mc->pin5_func == PIN_FUNC_HOME && pin5) return 1;
    return 0;
}

int16_t MotorControl_GetPWM(void)
{
    return current_pwm_output;
}

void MotorControl_Init(MotorControl_t *mc,
                       float pos_Kp, float pos_Ki, float pos_Kd,
                       float spd_Kp, float spd_Ki, float spd_Kd,
                       int16_t dead_zone, int16_t max_output)
{
    motor_ctl = mc;
    mc->pos_Kp = pos_Kp;
    mc->pos_Ki = pos_Ki;
    mc->pos_Kd = pos_Kd;
    mc->spd_Kp = spd_Kp;
    mc->spd_Ki = spd_Ki;
    mc->spd_Kd = spd_Kd;
    mc->dead_zone = dead_zone;
    mc->max_output = max_output;
    mc->direction = MOTOR_DIR_CW;
    mc->encoder_dir = ENCODER_DIR_NORMAL;
    mc->start_mode = 0;
    mc->modbus_addr = 1;
    mc->modbus_baud = 115200;
    mc->home_mode = HOME_MODE_OFF;
    mc->home_dir = 0;
    mc->home_current = 300;
    mc->home_speed = 1000;
    mc->home_max_distance = 10000;
    mc->home_back_distance = 100;
    mc->home_auto_start = 1;
    mc->home_precision_speed = 100;
    mc->home_precision_cycles = 3;
    mc->pin4_func = PIN_FUNC_PULSE;
    mc->pin4_polarity = POLARITY_ACTIVE_HIGH;
    mc->pin4_limit_dir = 0;
    mc->pin5_func = PIN_FUNC_DIR;
    mc->pin5_polarity = POLARITY_ACTIVE_HIGH;
    mc->pin5_limit_dir = 1;
    mc->max_run_speed = 0;
    mc->pin4_target_pos = 0;
    mc->pin5_target_pos = 0;
    mc->pin4_target_speed = 0;
    mc->pin5_target_speed = 0;
    mc->mode = POSITION_MODE;
    /* 从Flash加载保存的配置，覆盖默认值（无有效数据时保留默认值） */
    MotorControl_LoadConfig(mc);
    /* 初始化边沿检测状态（加载配置后根据极性初始化，避免首次触发虚假脉冲） */
    prev_pin4_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET ? 1 : 0;
    prev_pin5_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET ? 1 : 0;
    if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) prev_pin4_state = !prev_pin4_state;
    if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) prev_pin5_state = !prev_pin5_state;
    mc->target_position = 0;
    mc->target_speed = 0.0f;
    ramped_target_speed = 0.0f;
    mc->target_pos_frac = 0.0f;
    mc->integral = 0;
    mc->prev_error = 0;
    mc->last_tick = HAL_GetTick();
    mc->prev_position = 0;
    mc->velocity = 0.0f;
    mc->prev_velocity = 0.0f;
    /* 清零滤波缓冲区 */
    mc->speed_buf_idx = 0;
    mc->speed_sum = 0.0f;
    for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        mc->speed_buf[i] = 0.0f;
    }
}

static void MotorControl_ClearMotionState(MotorControl_t *mc)
{
    mc->target_position = 0;
    mc->target_speed = 0.0f;
    ramped_target_speed = 0.0f;
    mc->target_pos_frac = 0.0f;
    mc->prev_position = 0;
    mc->integral = 0;
    mc->prev_error = 0;
    mc->prev_velocity = 0.0f;
    mc->speed_sum = 0.0f;
    mc->speed_buf_idx = 0;
    speed_limit_integral = 0.0f;
    speed_limit_prev_velocity = 0.0f;
    for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        mc->speed_buf[i] = 0.0f;
    }
}

static void MotorControl_FinishHoming(MotorControl_t *mc, uint8_t failed)
{
    homing_failed = failed;
    homing_state = HOMING_IDLE;
    Encoder_Reset();
    MotorControl_ClearMotionState(mc);
    mc->mode = mode_before_homing;
    Motor_SetPWM(0);
}

void MotorControl_StartHoming(MotorControl_t *mc)
{
    if (mc->home_mode == HOME_MODE_STALL) {
        if (mc->home_current < 10 || mc->home_current > 1000) mc->home_current = 300;
        if (mc->home_speed <= 0) mc->home_speed = 1000;
        if (mc->home_max_distance <= 0) mc->home_max_distance = 10000;
        homing_state = HOMING_SEARCH;
        homing_failed = 0;
        homing_ticks = 0;
        homing_stall_ticks = 0;
        homing_start_position = Encoder_GetCount();
        mode_before_homing = mc->mode;

        mc->mode = VELOCITY_POSITION_MODE;
        mc->target_position = homing_start_position;
        mc->target_speed = (mc->home_dir != 0) ? (float)mc->home_speed : -(float)mc->home_speed;
        mc->target_pos_frac = 0.0f;
        mc->integral = 0;
        mc->prev_error = 0;
        return;
    }

    if (mc->home_mode == HOME_MODE_SWITCH) {
        if (mc->home_current < 10 || mc->home_current > 1000) mc->home_current = 300;
        if (mc->home_speed <= 0) mc->home_speed = 1000;
        if (mc->home_max_distance <= 0) mc->home_max_distance = 10000;
        if (mc->home_precision_speed <= 0) mc->home_precision_speed = 100;
        if (mc->home_precision_cycles == 0) mc->home_precision_cycles = 3;

        homing_failed = 0;
        homing_ticks = 0;
        homing_stall_ticks = 0;
        homing_start_position = Encoder_GetCount();
        mode_before_homing = mc->mode;
        precision_cycle_idx = 0;
        switch_edge_sum = 0;
        switch_edge_pos = 0;

        if (ReadHomeSwitch(mc)) {
            homing_state = HOMING_SWITCH_RELEASE;
        } else {
            homing_state = HOMING_SWITCH_SEARCH;
        }

        mc->mode = VELOCITY_POSITION_MODE;
        mc->target_position = Encoder_GetCount();
        mc->target_pos_frac = 0.0f;
        mc->integral = 0;
        mc->prev_error = 0;
        return;
    }

    return;
}

uint8_t MotorControl_IsHoming(void)
{
    return homing_state != HOMING_IDLE;
}

uint8_t MotorControl_HomingFailed(void)
{
    return homing_failed;
}

void MotorControl_SetTarget(MotorControl_t *mc, int64_t target)
{
    mc->target_position = target;
    mc->integral = 0;
    mc->prev_error = 0;
}

void MotorControl_SetSpeed(MotorControl_t *mc, float speed)
{
    mc->target_speed = speed;
}

void MotorControl_SetMode(MotorControl_t *mc, MotorMode_t mode)
{
    if (mode > EXTERNAL_TARGET_SPEED_MODE) {
        return;
    }

    if (mc->mode != mode) {
        if (mode == POSITION_MODE || mode == VELOCITY_POSITION_MODE || mode == EXTERNAL_TARGET_MODE) {
            mc->target_position = Encoder_GetCount();
            mc->target_pos_frac = 0.0f;
            mc->target_speed = 0.0f;
        } else if (mode == SPEED_MODE || mode == OPENLOOP_MODE || mode == EXTERNAL_TARGET_SPEED_MODE) {
            mc->target_speed = 0.0f;
            ramped_target_speed = mc->velocity;
        }
    }
    mc->mode = mode;
    mc->integral = 0;
    mc->prev_error = 0;
    speed_limit_integral = 0.0f;
    speed_limit_prev_velocity = mc->velocity;
}

void MotorControl_SetDirection(MotorControl_t *mc, MotorDir_t dir)
{
    mc->direction = dir;
}

void MotorControl_SetEncoderDirection(MotorControl_t *mc, EncoderDir_t dir)
{
    mc->encoder_dir = dir;
}

void MotorControl_SetPosPID(MotorControl_t *mc, float Kp, float Ki, float Kd)
{
    mc->pos_Kp = Kp;
    mc->pos_Ki = Ki;
    mc->pos_Kd = Kd;
    mc->integral = 0;
    mc->prev_error = 0;
}

void MotorControl_SetSpdPID(MotorControl_t *mc, float Kp, float Ki, float Kd)
{
    mc->spd_Kp = Kp;
    mc->spd_Ki = Ki;
    mc->spd_Kd = Kd;
    mc->integral = 0;
    mc->prev_error = 0;
}

void MotorControl_SetMaxOutput(MotorControl_t *mc, int16_t max_output)
{
    if (max_output > 1000) max_output = 1000;
    if (max_output < 10) max_output = 10;  /* 下限10, 防止误设为0导致电机不动 */
    mc->max_output = max_output;
}

void MotorControl_SetMaxRunSpeed(MotorControl_t *mc, int32_t max_run_speed)
{
    if (max_run_speed < 0 || max_run_speed > 1000000) max_run_speed = 0;
    mc->max_run_speed = max_run_speed;
    speed_limit_integral = 0.0f;
    speed_limit_prev_velocity = mc->velocity;
}

/**
  * @brief  设置当前位置为原点
  *         编码器清零，并同步电机控制器的位置/积分/速度滤波状态，
  *         防止下一次PID周期出现位置目标突变或速度尖峰。
  *         通过关闭TIM1中断实现临界区保护，避免与PID运算并发。
  */
void MotorControl_SetOrigin(MotorControl_t *mc)
{
    /* 关闭TIM1中断，避免在重置过程中PID中断读到不一致的状态 */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

    Encoder_Reset();            /* 编码器计数清零 */
    mc->target_position = 0;    /* 目标位置同步为0，防止突然转动 */
    mc->target_pos_frac = 0.0f;
    mc->prev_position = 0;      /* 防止下一次速度计算出现巨大尖峰 */
    mc->integral = 0;           /* 清零积分项 */
    mc->prev_error = 0;
    mc->prev_velocity = 0.0f;
    speed_limit_integral = 0.0f;
    speed_limit_prev_velocity = 0.0f;
    ramped_target_speed = 0.0f;
    /* 清零速度移动平均滤波缓冲 */
    mc->speed_sum = 0.0f;
    mc->speed_buf_idx = 0;
    for (int i = 0; i < SPEED_FILTER_SIZE; i++) {
        mc->speed_buf[i] = 0.0f;
    }

    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

static int16_t MotorControl_ApplyLimitSwitches(MotorControl_t *mc, int32_t output)
{
    uint8_t pin4 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t pin5 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
    if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) pin4 = !pin4;
    if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) pin5 = !pin5;
    if (mc->pin4_func == PIN_FUNC_LIMIT && pin4) {
        if (mc->pin4_limit_dir == 0 && output > 0) output = 0;
        if (mc->pin4_limit_dir == 1 && output < 0) output = 0;
    }
    if (mc->pin5_func == PIN_FUNC_LIMIT && pin5) {
        if (mc->pin5_limit_dir == 0 && output > 0) output = 0;
        if (mc->pin5_limit_dir == 1 && output < 0) output = 0;
    }

    return (int16_t)output;
}

int16_t MotorControl_Update(MotorControl_t *mc, int64_t current_position)
{
    /* 强制 dt = 5ms (由TIM1 100us中断计数50次驱动), 不依赖HAL_GetTick避免阻塞造成误差 */
    const float dt = 0.005f;

    /* 运行时保护: max_output 异常时强制使用默认值，防止PWM被限幅到0 */
    if (mc->max_output < 10 || mc->max_output > 1000) {
        mc->max_output = 900;
    }
    if (mc->max_run_speed < 0 || mc->max_run_speed > 1000000) {
        mc->max_run_speed = 0;
    }

    int16_t output_limit = mc->max_output;
    if (homing_state != HOMING_IDLE) {
        output_limit = mc->home_current;
        if (output_limit < 10 || output_limit > 1000) {
            output_limit = 300;
        }
    }

    /* 原始速度计算 */
    float raw_velocity = (float)(current_position - mc->prev_position) / dt;
    mc->prev_position = current_position;

    /* 移动平均滤波 */
    mc->speed_sum -= mc->speed_buf[mc->speed_buf_idx];
    mc->speed_buf[mc->speed_buf_idx] = raw_velocity;
    mc->speed_sum += raw_velocity;
    mc->speed_buf_idx = (mc->speed_buf_idx + 1) % SPEED_FILTER_SIZE;
    mc->velocity = mc->speed_sum / SPEED_FILTER_SIZE;
    current_velocity = mc->velocity;

    if (!motor_running) {
        ramped_target_speed = 0.0f;
        Motor_SetPWM(0);
        return 0;
    }

    if (homing_state == HOMING_SEARCH) {
        homing_ticks++;

        /* 复位开关检测: 如果配置了HOME功能的引脚有效，立即完成复位 */
        if (ReadHomeSwitch(mc)) {
            MotorControl_FinishHoming(mc, 0);
            return 0;
        }

        int64_t distance = abs64(current_position - homing_start_position);
        if (distance >= mc->home_max_distance) {
            MotorControl_FinishHoming(mc, 1);
            return 0;
        }

        int32_t effective_home_speed = mc->home_speed;
        if (mc->max_run_speed > 0 && effective_home_speed > mc->max_run_speed) {
            effective_home_speed = mc->max_run_speed;
        }
        int32_t stall_speed = effective_home_speed / 10;
        if (stall_speed < 5) stall_speed = 5;
        if (homing_ticks > HOMING_STALL_GRACE_TICKS && fabsf(mc->velocity) <= (float)stall_speed) {
            homing_stall_ticks++;
        } else {
            homing_stall_ticks = 0;
        }

        if (homing_stall_ticks >= HOMING_STALL_TICKS) {
            Encoder_Reset();
            MotorControl_ClearMotionState(mc);
            if (mc->home_back_distance == 0) {
                MotorControl_FinishHoming(mc, 0);
                return 0;
            }
            homing_state = HOMING_BACK;
            int32_t back_dir = (mc->home_dir != 0) ? -1 : 1;
            mc->mode = POSITION_MODE;
            mc->target_position = (int64_t)back_dir * mc->home_back_distance;
            Motor_SetPWM(0);
            return 0;
        }
    }

    if (homing_state == HOMING_BACK) {
        int64_t err = mc->target_position - current_position;
        if (abs64(err) <= 2) {
            MotorControl_FinishHoming(mc, 0);
            return 0;
        }
    }

    /* ========== 开关精确复位状态机 ========== */
    if (homing_state == HOMING_SWITCH_SEARCH) {
        homing_ticks++;
        if (ReadHomeSwitch(mc)) {
            homing_state = HOMING_SWITCH_RELEASE;
            mc->target_speed = (float)mc->home_speed;
            mc->integral = 0;
            mc->prev_error = 0;
            Motor_SetPWM(0);
            return 0;
        }
        int64_t distance = abs64(current_position - homing_start_position);
        if (distance >= mc->home_max_distance) {
            MotorControl_FinishHoming(mc, 1);
            return 0;
        }
        mc->target_speed = -(float)mc->home_speed;
    }

    if (homing_state == HOMING_SWITCH_RELEASE) {
        homing_ticks++;
        if (!ReadHomeSwitch(mc)) {
            switch_edge_pos = current_position;
            homing_state = HOMING_SWITCH_PRECISION_APPROACH;
            precision_cycle_idx = 0;
            switch_edge_sum = 0;
            mc->target_speed = -(float)mc->home_precision_speed;
            mc->integral = 0;
            mc->prev_error = 0;
            Motor_SetPWM(0);
            return 0;
        }
        int64_t distance = abs64(current_position - homing_start_position);
        if (distance >= mc->home_max_distance) {
            MotorControl_FinishHoming(mc, 1);
            return 0;
        }
        mc->target_speed = (float)mc->home_speed;
    }

    if (homing_state == HOMING_SWITCH_PRECISION_APPROACH) {
        homing_ticks++;
        if (ReadHomeSwitch(mc)) {
            switch_edge_pos = current_position;
            switch_edge_sum += current_position;
            homing_state = HOMING_SWITCH_PRECISION_RELEASE;
            mc->target_speed = (float)mc->home_precision_speed;
            mc->integral = 0;
            mc->prev_error = 0;
            Motor_SetPWM(0);
            return 0;
        }
        int64_t distance = abs64(current_position - homing_start_position);
        if (distance >= mc->home_max_distance) {
            MotorControl_FinishHoming(mc, 1);
            return 0;
        }
        mc->target_speed = -(float)mc->home_precision_speed;
    }

    if (homing_state == HOMING_SWITCH_PRECISION_RELEASE) {
        homing_ticks++;
        if (!ReadHomeSwitch(mc)) {
            switch_edge_sum += current_position;
            precision_cycle_idx++;
            if (precision_cycle_idx < mc->home_precision_cycles) {
                homing_state = HOMING_SWITCH_PRECISION_APPROACH;
                mc->target_speed = -(float)mc->home_precision_speed;
            } else {
                homing_state = HOMING_SWITCH_BACK;
                /* 计算平均边缘位置 */
                int32_t total_edges = (int32_t)precision_cycle_idx * 2;
                int64_t avg_edge = switch_edge_sum / total_edges;
                mc->mode = POSITION_MODE;
                mc->target_position = avg_edge + (int64_t)mc->home_back_distance;
                Motor_SetPWM(0);
                return 0;
            }
            mc->integral = 0;
            mc->prev_error = 0;
            Motor_SetPWM(0);
            return 0;
        }
        int64_t distance = abs64(current_position - homing_start_position);
        if (distance >= mc->home_max_distance) {
            MotorControl_FinishHoming(mc, 1);
            return 0;
        }
        mc->target_speed = (float)mc->home_precision_speed;
    }

    if (homing_state == HOMING_SWITCH_BACK) {
        int64_t err = mc->target_position - current_position;
        if (abs64(err) <= 2) {
            MotorControl_FinishHoming(mc, 0);
            return 0;
        }
    }

    /* 外部目标位置模式：PB4优先于PB5；都无效时回到原点位置 */
    if (mc->mode == EXTERNAL_TARGET_MODE &&
        (mc->pin4_func == PIN_FUNC_TARGET || mc->pin5_func == PIN_FUNC_TARGET)) {
        uint8_t pin4 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t pin5 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
        if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) pin4 = !pin4;
        if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) pin5 = !pin5;

        uint8_t input = 0;
        int64_t target = 0;
        if (mc->pin4_func == PIN_FUNC_TARGET && pin4) {
            input = 4;
            target = mc->pin4_target_pos;
        } else if (mc->pin5_func == PIN_FUNC_TARGET && pin5) {
            input = 5;
            target = mc->pin5_target_pos;
        }

        if (target_level_state != input || mc->target_position != target) {
            target_level_state = input;
            MotorControl_SetTarget(mc, target);
            mc->target_pos_frac = 0.0f;
            MODBUS_SyncTargetPosition(mc->target_position);
        }
    }

    /* 外部目标速度模式：PB4优先于PB5；都无效时停止 */
    if (mc->mode == EXTERNAL_TARGET_SPEED_MODE &&
        (mc->pin4_func == PIN_FUNC_TARGET || mc->pin5_func == PIN_FUNC_TARGET)) {
        uint8_t pin4 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t pin5 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
        if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) pin4 = !pin4;
        if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) pin5 = !pin5;

        uint8_t input = 0;
        float target = 0.0f;
        if (mc->pin4_func == PIN_FUNC_TARGET && pin4) {
            input = 4;
            target = (float)mc->pin4_target_speed;
        } else if (mc->pin5_func == PIN_FUNC_TARGET && pin5) {
            input = 5;
            target = (float)mc->pin5_target_speed;
        }

        if (target_level_state != input || mc->target_speed != target) {
            target_level_state = input;
            MotorControl_SetSpeed(mc, target);
        }
    }

    /* 开环模式：目标速度直接映射到PWM输出，受output_limit和最大运行速度限制 */
    if (mc->mode == OPENLOOP_MODE) {
        int32_t output = (int32_t)mc->target_speed;
        if (output > output_limit) output = output_limit;
        if (output < -output_limit) output = -output_limit;
        if (mc->max_run_speed > 0 &&
            ((mc->velocity >= (float)mc->max_run_speed && output > 0) ||
             (mc->velocity <= -(float)mc->max_run_speed && output < 0))) {
            output = 0;
        }
        output = MotorControl_ApplyLimitSwitches(mc, output);
        Motor_SetPWM((int16_t)output);
        return (int16_t)output;
    }

    /* 脉冲累加：将TIM2中断中检测到的脉冲边缘累加到目标位置 */
    if (pulse_accumulator != 0) {
        __disable_irq();
        int32_t pulses = pulse_accumulator;
        pulse_accumulator = 0;
        __enable_irq();
        mc->target_position += pulses;
        MODBUS_SyncTargetPosition(mc->target_position);
    }

    /* 基于位置的速度模式：目标位置按速度累加（浮点累加避免低速截断） */
    if (mc->mode == VELOCITY_POSITION_MODE) {
        float limited_speed = mc->target_speed;
        if (mc->max_run_speed > 0) limited_speed = clampf(limited_speed, -(float)mc->max_run_speed, (float)mc->max_run_speed);
        mc->target_pos_frac += limited_speed * dt;
        int64_t inc = (int64_t)mc->target_pos_frac;
        mc->target_position += inc;
        mc->target_pos_frac -= (float)inc;
        MODBUS_SyncTargetPosition(mc->target_position);
    }

    float error;
    if (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE) {
        float limited_speed = mc->target_speed;
        if (mc->max_run_speed > 0) limited_speed = clampf(limited_speed, -(float)mc->max_run_speed, (float)mc->max_run_speed);
        limited_speed = MotorControl_UpdateSpeedRamp(limited_speed, dt);
        error = limited_speed - mc->velocity;
        /* 速度死区: 误差小时不加积分, 防止抖动 */
        if (fabsf(error) < 20.0f) {
            error = 0;
        }
    } else {
        error = (float)(mc->target_position - current_position);
        /* 位置模式/外部目标位置模式走死区；位置速度模式不走死区（目标持续移动，死区会导致脉冲式运动） */
        if ((mc->mode == POSITION_MODE || mc->mode == EXTERNAL_TARGET_MODE) && fabsf(error) <= mc->dead_zone) {
            mc->integral = 0;
            mc->prev_error = 0;
            speed_limit_integral = 0.0f;
            speed_limit_prev_velocity = mc->velocity;
            Motor_SetPWM(0);
            return 0;
        }
    }

    if ((mc->mode == POSITION_MODE || mc->mode == EXTERNAL_TARGET_MODE) && mc->max_run_speed > 0) {
        float target_velocity = mc->pos_Kp * error;
        target_velocity = clampf(target_velocity, -(float)mc->max_run_speed, (float)mc->max_run_speed);
        float speed_error = target_velocity - mc->velocity;

        speed_limit_integral += speed_error * dt;
        float P = mc->spd_Kp * speed_error;
        float I = mc->spd_Ki * speed_limit_integral;
        float D = -mc->spd_Kd * (mc->velocity - speed_limit_prev_velocity) / dt;
        mc->pid_error = speed_error;
        mc->pid_p = P;
        mc->pid_i = I;
        mc->pid_d = D;
        mc->prev_error = error;
        speed_limit_prev_velocity = mc->velocity;
        mc->prev_velocity = mc->velocity;

        int32_t output = (int32_t)(P + I + D);
        if (output > output_limit) {
            output = output_limit;
            speed_limit_integral -= speed_error * dt;
            mc->pid_i = mc->spd_Ki * speed_limit_integral;
        }
        if (output < -output_limit) {
            output = -output_limit;
            speed_limit_integral -= speed_error * dt;
            mc->pid_i = mc->spd_Ki * speed_limit_integral;
        }

        output = MotorControl_ApplyLimitSwitches(mc, output);
        Motor_SetPWM((int16_t)output);
        return (int16_t)output;
    }

    float Kp = (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE) ? mc->spd_Kp : mc->pos_Kp;
    float Ki = (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE) ? mc->spd_Ki : mc->pos_Ki;
    float Kd = (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE) ? mc->spd_Kd : mc->pos_Kd;

    float P = Kp * error;
    mc->pid_error = error;

    /* 位置模式: 误差过零时清零积分，防止超调后积分反向抵消导致缓慢回正 */
    /* VELOCITY_POSITION_MODE 跳过此逻辑：跟踪中误差持续变化，清零积分会导致抖动 */
    if (mc->mode == POSITION_MODE || mc->mode == EXTERNAL_TARGET_MODE) {
        if (mc->prev_error != 0.0f &&
            ((mc->prev_error > 0.0f && error < 0.0f) ||
             (mc->prev_error < 0.0f && error > 0.0f))) {
            mc->integral = 0;
        }
    }

    /* 标准PID积分: integral累加 error*dt, I = Ki * integral */
    mc->integral += error * dt;
    float I = Ki * mc->integral;
    mc->pid_p = P;
    mc->pid_i = I;

    /* 速度模式D项使用速度变化率, 减小噪声放大 */
    float D;
    if (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE) {
        /* D项基于速度变化, 经过移动平均后噪声小 */
        D = -Kd * (mc->velocity - mc->prev_velocity) / dt;
    } else {
        /* 位置模式: 用当前速度做微分阻尼 */
        D = -Kd * mc->velocity;
    }
    mc->pid_d = D;
    mc->prev_error = error;
    mc->prev_velocity = mc->velocity;

    int32_t output = (int32_t)(P + I + D);

    /* 输出限幅 + 抗积分饱和: 输出饱和时停止积分继续累积 */
    if (output > output_limit) {
        output = output_limit;
        /* 饱和时回退本次积分累积 */
        mc->integral -= error * dt;
        mc->pid_i = Ki * mc->integral;
    }
    if (output < -output_limit) {
        output = -output_limit;
        /* 饱和时回退本次积分累积 */
        mc->integral -= error * dt;
        mc->pid_i = Ki * mc->integral;
    }

    /* 限位开关钳制：激活的限位开关阻止该方向运动，只允许反向 */
    output = MotorControl_ApplyLimitSwitches(mc, output);

    Motor_SetPWM((int16_t)output);
    return (int16_t)output;
}

void MotorControl_Reset(MotorControl_t *mc)
{
    mc->integral = 0;
    mc->prev_error = 0;
    mc->last_tick = HAL_GetTick();
    Motor_SetPWM(0);
}

int64_t MotorControl_GetTarget(MotorControl_t *mc)
{
    return mc->target_position;
}

float MotorControl_GetSpeedTarget(MotorControl_t *mc)
{
    return mc->target_speed;
}

MotorMode_t MotorControl_GetMode(MotorControl_t *mc)
{
    return mc->mode;
}

void Motor_Start(void)
{
    motor_running = 1;

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    TIM4->CCR2 = 1000;
    TIM4->CCR4 = 1000;

    /* 启动前清零编码器 */
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    encoder_wrap = 0;
    last_valid_count = 0;

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* 清除初始化时残留的更新标志，再使能中断 */
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    /* TIM3编码器溢出中断优先级设为最高(0)，确保编码器计数原子性
     * 高于TIM1(PID,优先级2)/TIM2(脉冲,优先级2)/USART1(MODBUS,优先级2) */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void Motor_Stop(void)
{
    motor_running = 0;
    Motor_SetPWM(0);
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

void Motor_SetPWM(int16_t output)
{
    /* 记录当前原始PWM输出（不含方向取反，便于观察PID输出）
     * 范围: -1000~+1000, 对应 -100%~+100% */
    current_pwm_output = output;

    /* 电机方向反转时，输出极性取反 */
    if (motor_ctl && motor_ctl->direction == MOTOR_DIR_CCW) {
        output = -output;
    }

    if (output > 0) {
        TIM4->CCR1 = output;
        TIM4->CCR3 = 0;
    } else if (output < 0) {
        TIM4->CCR1 = 0;
        TIM4->CCR3 = -output;
    } else {
        TIM4->CCR1 = 0;
        TIM4->CCR3 = 0;
    }
}

void Encoder_OverflowHandler(void)
{
    if ((TIM3->CR1 & TIM_CR1_DIR) == 0) {
        encoder_wrap++;
    } else {
        encoder_wrap--;
    }
}

int64_t Encoder_GetCount(void)
{
    /* seqlock模式: 读取前后比较encoder_wrap，检测读取过程中是否发生TIM3溢出中断
     * TIM3优先级最高(0)，在TIM1(PID)中断中调用本函数时TIM3仍可抢占执行，
     * 因此wrap可能在读取CNT前后变化。若变化说明刚好中断来袭，跳过本轮合成，
     * 返回上一次有效值，避免合成出错误计数(如 0*65536+小值 而非 1*65536+小值)。 */
    int32_t wrap1 = encoder_wrap;
    uint16_t cnt = (uint16_t)TIM3->CNT;  /* 必须用uint16_t: CNT是0~65535无符号值, 误用int16_t会让>=32768的值变负数导致跳变65536 */
    int32_t wrap2 = encoder_wrap;

    int64_t count;
    if (wrap1 != wrap2) {
        /* 读取过程中发生了TIM3溢出中断，跳过本轮合成，返回上一次有效值 */
        count = last_valid_count;
    } else {
        count = (int64_t)wrap1 * 65536 + cnt;
        last_valid_count = count;
    }

    /* 编码器方向反转时计数取反 */
    if (motor_ctl && motor_ctl->encoder_dir == ENCODER_DIR_REVERSED) {
        count = -count;
    }
    return count;
}

float Encoder_GetSpeed(void)
{
    float spd = current_velocity;
    /* 编码器方向反转时速度取反 */
    if (motor_ctl && motor_ctl->encoder_dir == ENCODER_DIR_REVERSED) {
        spd = -spd;
    }
    return spd;
}

void Encoder_Reset(void)
{
    TIM3->CNT = 0;
    encoder_wrap = 0;
    last_valid_count = 0;
}

/* 主循环中处理PB4/PB5输入（备份检查，定时器中断已处理脉冲/限位） */
void MotorControl_ProcessInputs(void)
{
    if (!motor_ctl) return;
}

/* TIM2 10us中断中调用：脉冲输入边沿检测（高速采样避免丢脉冲） */
void MotorControl_PulseTick10us(void)
{
    if (motor_ctl) {
        uint8_t current_pin4 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t current_pin5 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;

        if (motor_ctl->pin4_polarity == POLARITY_ACTIVE_LOW) current_pin4 = !current_pin4;
        if (motor_ctl->pin5_polarity == POLARITY_ACTIVE_LOW) current_pin5 = !current_pin5;

        /* 查找方向引脚 */
        uint8_t dir_pin_state = 0;
        if (motor_ctl->pin4_func == PIN_FUNC_DIR) {
            dir_pin_state = current_pin4;
        } else if (motor_ctl->pin5_func == PIN_FUNC_DIR) {
            dir_pin_state = current_pin5;
        }

        /* PB4 脉冲边沿 */
        if (current_pin4 && !prev_pin4_state) {
            if (motor_ctl->pin4_func == PIN_FUNC_PULSE) {
                pulse_accumulator += dir_pin_state ? 1 : -1;
            }
        }
        prev_pin4_state = current_pin4;

        /* PB5 脉冲边沿 */
        if (current_pin5 && !prev_pin5_state) {
            if (motor_ctl->pin5_func == PIN_FUNC_PULSE) {
                pulse_accumulator += dir_pin_state ? 1 : -1;
            }
        }
        prev_pin5_state = current_pin5;
    }
}

/* TIM1 100us滴答，每50次(约5ms)执行一次PID */
void MotorControl_TimerTick100us(void)
{
    static uint16_t pid_divider = 0;

    pid_divider++;
    if (pid_divider >= 50) {
        pid_divider = 0;
        if (motor_ctl) {
            MotorControl_Update(motor_ctl, Encoder_GetCount());
        }
    }
}
