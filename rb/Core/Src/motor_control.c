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
 *   溢出中断: 已关闭. 64位位置由TIM1 100μs中断做差分累加合成
 *
 * TIM1 (100us中断, 内部计数驱动PID)
 *   PSC=63, ARR=99 -> ~89us中断
 *   每次执行: Encoder_Synthesize() 差分累加TIM3->CNT到64位计数
 *   每50次调用(约4.5ms)执行一次PID: MotorControl_Update()
 *
 * ============ 中断处理 ============
 *
 * 以下中断处理函数由本库提供:
 *   MotorControl_TimerTick100us - 由TIM1 100us中断调用
 *     每次执行: 编码器位置合成(Encoder_Synthesize)
 *     内部每50次触发一次 MotorControl_Update()
 *
 * 以下中断处理函数需要用户在 stm32f1xx_it.c 中实现:
 *   TIM1_UP_IRQHandler -> 调用 MotorControl_TimerTick100us()
 *   (TIM3_IRQHandler不再需要处理溢出, TIM3溢出中断已关闭)
 *
 * 调用顺序:
 *   main() -> MotorControl_Init() -> Motor_Start()
 *
 *   TIM1每~89us触发 -> TIM1_UP_IRQHandler
 *                    -> MotorControl_TimerTick100us()
 *                       ├─ Encoder_Synthesize() 读CNT, 差分累加到encoder_total_count
 *                       └─ 每50次: MotorControl_Update() -> Motor_SetPWM()
 *
 *   编码器位置读取(任何上下文):
 *     Encoder_GetCount() 关全局中断读encoder_total_count(64位原子读)
 */

#include "motor_control.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

extern void MODBUS_SyncTargetPosition(int64_t position);
extern void MODBUS_TriggerStartFlag(void);  /* PB4/PB5启动标志位触发时调用 */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;  /* 电流采样ADC (ADC1, PC1, DMA连续转换) */

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
    /* 电流环参数(新增) */
    uint32_t cur_kp;
    uint32_t cur_ki;
    uint32_t cur_kd;
    uint32_t current_offset;
    uint32_t current_scale;
    uint32_t current_loop_en;
    uint32_t over_current_limit;
    /* PC2/PC3 ADC功能参数(新增) */
    uint32_t pc2_func;
    uint32_t pc3_func;
    uint32_t adc_max_speed;
    uint32_t adc_max_pwm;
    uint32_t adc_max_position_h;  /* int64高32位 */
    uint32_t adc_max_position_l;  /* int64低32位 */
    /* ADC最小值参数(新增) - 支持负数, 有符号值按uint32_t存储 */
    uint32_t adc_min_speed;
    uint32_t adc_min_pwm;
    uint32_t adc_min_position_h;  /* int64高32位 */
    uint32_t adc_min_position_l;  /* int64低32位 */
    /* ADC死区参数(新增) */
    uint32_t adc_dead_zone1_pos;
    uint32_t adc_dead_zone1_width;
    uint32_t adc_dead_zone2_pos;
    uint32_t adc_dead_zone2_width;
    /* 堵转保护参数 */
    uint32_t stall_protect_en;
    uint32_t stall_err_limit;
    uint32_t stall_time_ticks;
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
        if (cfg->mode <= ADC_POSITION_MODE) {
            mc->mode = (MotorMode_t)cfg->mode;
        }
        /* PB4/PB5功能枚举: 早期版本曾拆分TARGET为TARGET_POS/TARGET_SPEED,
         * 旧迁移逻辑会误把新枚举值5(TARGET_SPEED)当作旧HOME_START迁移为6, 反而导致"改完不保存".
         * 现直接按当前枚举加载, 从旧固件升级的用户需重新配置一次PB4/PB5功能即可 */
        if (cfg->pin4_func <= PIN_FUNC_START_FLAG) {
            mc->pin4_func = (uint8_t)cfg->pin4_func;
        }
        if (cfg->pin4_polarity <= 1) {
            mc->pin4_polarity = (uint8_t)cfg->pin4_polarity;
        }
        if (cfg->pin4_limit_dir <= 1) {
            mc->pin4_limit_dir = (uint8_t)cfg->pin4_limit_dir;
        }
        if (cfg->pin5_func <= PIN_FUNC_START_FLAG) {
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
        /* 加载电流环参数(新增): NaN检查+范围检查, 无效时保留默认值 */
        f = u32_to_float(cfg->cur_kp);
        if (f == f && f >= 0.0f) mc->cur_Kp = f;
        f = u32_to_float(cfg->cur_ki);
        if (f == f && f >= 0.0f) mc->cur_Ki = f;
        f = u32_to_float(cfg->cur_kd);
        if (f == f && f >= 0.0f) mc->cur_Kd = f;
        if (cfg->current_offset <= 4095) {
            mc->current_offset = (uint16_t)cfg->current_offset;
        }
        f = u32_to_float(cfg->current_scale);
        if (f == f && f > 0.0f) mc->current_scale = f;
        mc->current_loop_en = (cfg->current_loop_en != 0) ? 1 : 0;
        f = u32_to_float(cfg->over_current_limit);
        if (f == f && f >= 0.0f) mc->over_current_limit = f;

        if (cfg->target_pos_magic == FLASH_TARGET_MAGIC) {
            mc->pin4_target_pos = (int64_t)(((uint64_t)cfg->pin4_target_pos_h << 32) | cfg->pin4_target_pos_l);
            mc->pin5_target_pos = (int64_t)(((uint64_t)cfg->pin5_target_pos_h << 32) | cfg->pin5_target_pos_l);
        }
        /* 加载PC2/PC3 ADC功能参数(新增) */
        if (cfg->pc2_func <= ADC_FUNC_POSITION) {
            mc->pc2_func = (uint8_t)cfg->pc2_func;
        }
        if (cfg->pc3_func <= ADC_FUNC_POSITION) {
            mc->pc3_func = (uint8_t)cfg->pc3_func;
        }
        if (cfg->adc_max_speed <= 1000000) {
            mc->adc_max_speed = (int32_t)cfg->adc_max_speed;
        }
        if (cfg->adc_max_pwm <= 1000) {
            mc->adc_max_pwm = (int16_t)(int32_t)cfg->adc_max_pwm;
        }
        mc->adc_max_position = (int64_t)(((uint64_t)cfg->adc_max_position_h << 32) | cfg->adc_max_position_l);
        /* 加载ADC最小值参数(新增) */
        mc->adc_min_speed = (int32_t)cfg->adc_min_speed;
        {
            int16_t min_pwm = (int16_t)(int32_t)cfg->adc_min_pwm;
            if (min_pwm >= -1000 && min_pwm <= 1000) {
                mc->adc_min_pwm = min_pwm;
            }
        }
        mc->adc_min_position = (int64_t)(((uint64_t)cfg->adc_min_position_h << 32) | cfg->adc_min_position_l);
        /* 加载ADC死区参数(新增) */
        if (cfg->adc_dead_zone1_pos <= 2) {
            mc->adc_dead_zone1_pos = (uint8_t)cfg->adc_dead_zone1_pos;
        }
        if (cfg->adc_dead_zone1_width <= 4095) {
            mc->adc_dead_zone1_width = (uint16_t)cfg->adc_dead_zone1_width;
        }
        if (cfg->adc_dead_zone2_pos <= 2) {
            mc->adc_dead_zone2_pos = (uint8_t)cfg->adc_dead_zone2_pos;
        }
        if (cfg->adc_dead_zone2_width <= 4095) {
            mc->adc_dead_zone2_width = (uint16_t)cfg->adc_dead_zone2_width;
        }
        /* 加载堵转保护参数 */
        mc->stall_protect_en = (cfg->stall_protect_en != 0) ? 1 : 0;
        mc->stall_err_limit = (int32_t)cfg->stall_err_limit;
        mc->stall_time_ticks = (uint16_t)cfg->stall_time_ticks;
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
    /* 保存电流环参数(新增) - 删除tim2_arr后所有字段前移4字节 */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 152, float_to_u32(mc->cur_Kp));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 156, float_to_u32(mc->cur_Ki));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 160, float_to_u32(mc->cur_Kd));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 164, (uint32_t)mc->current_offset);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 168, float_to_u32(mc->current_scale));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 172, (uint32_t)mc->current_loop_en);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 176, float_to_u32(mc->over_current_limit));
    /* 保存PC2/PC3 ADC功能参数(新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 180, (uint32_t)mc->pc2_func);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 184, (uint32_t)mc->pc3_func);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 188, (uint32_t)mc->adc_max_speed);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 192, (uint32_t)(int32_t)mc->adc_max_pwm);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 196, (uint32_t)((uint64_t)mc->adc_max_position >> 32));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 200, (uint32_t)((uint64_t)mc->adc_max_position & 0xFFFFFFFFULL));
    /* 保存ADC最小值参数(新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 204, (uint32_t)(int32_t)mc->adc_min_speed);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 208, (uint32_t)(int32_t)mc->adc_min_pwm);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 212, (uint32_t)((uint64_t)mc->adc_min_position >> 32));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 216, (uint32_t)((uint64_t)mc->adc_min_position & 0xFFFFFFFFULL));
    /* 保存ADC死区参数(新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 220, (uint32_t)mc->adc_dead_zone1_pos);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 224, (uint32_t)mc->adc_dead_zone1_width);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 228, (uint32_t)mc->adc_dead_zone2_pos);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 232, (uint32_t)mc->adc_dead_zone2_width);
    /* 保存堵转保护参数 */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 236, (uint32_t)mc->stall_protect_en);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 240, (uint32_t)mc->stall_err_limit);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 244, (uint32_t)mc->stall_time_ticks);

    HAL_FLASH_Lock();
}

static uint8_t motor_running = 0;
static float current_velocity = 0.0f;
static float ramped_target_speed = 0.0f;

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
/* ADC DMA采样缓冲区(4通道, DMA循环模式持续刷新)
 * 数组元素按ADC规则通道Rank顺序对应:
 *   [0] = Rank1 = ADC_CHANNEL_11 = PC1 (电流采样)
 *   [1] = Rank2 = ADC_CHANNEL_10 = PC0 (供电电压检测, 100K:10K分压, ×11)
 *   [2] = Rank3 = ADC_CHANNEL_12 = PC2 (外部ADC输入1, 电位器调速等)
 *   [3] = Rank4 = ADC_CHANNEL_13 = PC3 (外部ADC输入2, 电位器调速等)
 * 使用uint16_t匹配DMA的HALFWORD传输配置, 4次传输正好填满4个元素 */
static volatile uint16_t adc_values[4] = {0, 0, 0, 0};

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

/* 停止功能：按钮按下时PWM强制输出0，松开后恢复 */
static uint8_t stop_override = 0;  /* 0=正常, 1=停止(PWM=0) */

/* 前向声明: MotorControl_SetOutput 在 MotorControl_FinishHoming 之后定义,
 * 但 FinishHoming 需要调用它, 因此提前声明 */
static void MotorControl_SetOutput(MotorControl_t *mc, int16_t output);

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

/* 读取配置为原点位置开关(PIN_FUNC_HOME)的引脚状态（有效=1，无效=0）
 * 用于堵转复位终点检测或开关精确复位的边沿定位 */
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
    mc->pc2_func = ADC_FUNC_NONE;
    mc->pc3_func = ADC_FUNC_NONE;
    mc->adc_min_speed = 0;           /* 默认最小速度0 */
    mc->adc_max_speed = 10000;       /* 默认10000脉冲/秒 */
    mc->adc_min_pwm = 0;             /* 默认最小PWM 0 */
    mc->adc_max_pwm = 500;           /* 默认500(满量程50%) */
    mc->adc_min_position = 0;        /* 默认最小位置0 */
    mc->adc_max_position = 100000;   /* 默认100000脉冲 */
    mc->adc_dead_zone1_pos = 0;      /* 默认死区1位置=最小点 */
    mc->adc_dead_zone1_width = 0;    /* 默认死区1宽度=0(关闭) */
    mc->adc_dead_zone2_pos = 0;      /* 默认死区2位置=最小点 */
    mc->adc_dead_zone2_width = 0;    /* 默认死区2宽度=0(关闭) */
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
    /* 转速采集初始化 */
    mc->speed_acq_count = 0;
    mc->speed_acq_divider = 50;  /* 默认5ms (50×100us) */
    mc->speed_acq_div_cnt = 0;
    mc->speed_acq_active = 0;
    mc->speed_acq_done = 0;
    mc->speed_acq_type = 0;      /* 默认采集转速 */
    mc->speed_acq_size = SPEED_ACQ_BUF_SIZE;  /* 默认采满缓冲区 */

    /* 堵转保护初始化(默认关闭, 需用户启用) */
    mc->stall_protect_en = 0;
    mc->stall_err_limit = 200;       /* 默认200脉冲 / 200脉冲/秒 */
    mc->stall_time_ticks = 200;      /* 默认200个tick = 1秒持续 */
    mc->stall_tick_cnt = 0;
    mc->stall_tripped = 0;

    /* ===== 电流环参数初始化(新增) =====
     * 默认关闭电流环(current_loop_en=0), 行为与原代码完全一致.
     * 用户需配置电流采样硬件后, 通过MotorControl_EnableCurrentLoop开启.
     * 默认PID: Kp=0.5, Ki=50, Kd=0 (电流环Ki可较大, 因100μs周期积分慢)
     * 默认标定: offset=2048(12位ADC中点), scale=1000/2048≈0.488
     *   即 ADC值 0→电流-1000, ADC值 2048→电流0, ADC值 4095→电流+1000
     * 默认过流保护: 关闭(over_current_limit=0) */
    mc->cur_Kp = 0.5f;
    mc->cur_Ki = 50.0f;
    mc->cur_Kd = 0.0f;
    mc->current_target = 0.0f;
    mc->current_actual = 0.0f;
    mc->current_integral = 0.0f;
    mc->current_prev_error = 0.0f;
    mc->current_offset = 2048;
    mc->current_scale = 1000.0f / 2048.0f;
    mc->current_loop_en = 0;
    mc->over_current_tripped = 0;
    mc->over_current_limit = 0.0f;

    /* ===== 速度环独立周期参数初始化(新增) ===== */
    mc->target_velocity = 0.0f;
    mc->speed_loop_integral = 0.0f;
    mc->speed_loop_prev_vel = 0.0f;
    mc->speed_loop_active = 0;
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
    /* 清零电流环/速度环状态 */
    mc->current_target = 0.0f;
    mc->current_integral = 0.0f;
    mc->current_prev_error = 0.0f;
    mc->target_velocity = 0.0f;
    mc->speed_loop_integral = 0.0f;
    mc->speed_loop_active = 0;
}

static void MotorControl_FinishHoming(MotorControl_t *mc, uint8_t failed)
{
    homing_failed = failed;
    homing_state = HOMING_IDLE;
    Encoder_Reset();
    MotorControl_ClearMotionState(mc);
    mc->speed_loop_active = 0;
    mc->current_target = 0.0f;
    mc->mode = mode_before_homing;
    MotorControl_SetOutput(mc, 0);
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
    if (mode > ADC_POSITION_MODE) {
        return;
    }

    if (mc->mode != mode) {
        if (mode == POSITION_MODE || mode == VELOCITY_POSITION_MODE || mode == EXTERNAL_TARGET_MODE ||
            mode == ADC_POSITION_SPEED_MODE) {
            mc->target_position = Encoder_GetCount();
            mc->target_pos_frac = 0.0f;
            mc->target_speed = 0.0f;
        } else if (mode == SPEED_MODE || mode == OPENLOOP_MODE || mode == EXTERNAL_TARGET_SPEED_MODE ||
                   mode == ADC_SPEED_MODE) {
            mc->target_speed = 0.0f;
            ramped_target_speed = mc->velocity;
        } else if (mode == ADC_POSITION_MODE) {
            mc->target_position = 0;
            mc->target_pos_frac = 0.0f;
        }
    }
    mc->mode = mode;
    mc->integral = 0;
    mc->prev_error = 0;
    speed_limit_integral = 0.0f;
    speed_limit_prev_velocity = mc->velocity;
    /* 切换模式时清零电流环/速度环状态, 防止残留积分导致输出突变 */
    mc->current_target = 0.0f;
    mc->current_integral = 0.0f;
    mc->current_prev_error = 0.0f;
    mc->target_velocity = 0.0f;
    mc->speed_loop_integral = 0.0f;
    mc->speed_loop_active = 0;
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
    mc->speed_loop_integral = 0.0f;  /* 清零1ms速度环独立积分 */
    mc->speed_loop_prev_vel = mc->velocity;
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
    /* 清零电流环/速度环状态 */
    mc->current_target = 0.0f;
    mc->current_integral = 0.0f;
    mc->current_prev_error = 0.0f;
    mc->target_velocity = 0.0f;
    mc->speed_loop_integral = 0.0f;
    mc->speed_loop_active = 0;

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

/* ========== 电流环（100μs周期, 新增）==========
 *
 * 读取ADC电流采样值, 与current_target做PID, 输出PWM(-1000~+1000).
 * 电流环使能后由TIM1 100μs中断每次调用.
 * 电流环关闭时: 电流环函数返回current_target作为PWM(透传, 等效于原无电流环行为). */

/* 读取ADC电流采样值并转换为相对电流(-1000~+1000)
 * DMA循环模式下, adc_values[0]由DMA持续刷新, 直接读取即可 */
static float MotorControl_ReadCurrent(MotorControl_t *mc)
{
    uint16_t adc_val = (uint16_t)(adc_values[0] & 0x0FFFu);  /* 12位ADC, 屏蔽高4位 */
    float current = ((float)adc_val - (float)mc->current_offset) * mc->current_scale;
    return current;
}

/* 读取供电电压(单位:V), PC0分压100K:10K, 实际电压=ADC电压×11
 * ADC 12位, 参考电压3.3V, Vsupply = adc/4095 × 3.3 × 11 ≈ adc × 0.00886 */
float MotorControl_GetSupplyVoltage(void)
{
    uint16_t adc_val = (uint16_t)(adc_values[1] & 0x0FFFu);
    return (float)adc_val * 3.3f * 11.0f / 4095.0f;
}

/* 读取供电电压ADC原始值(12位, 0~4095, PC0通道)
 * 供数据采集使用, 避免重复浮点运算 */
uint16_t MotorControl_GetSupplyVoltageADC(void)
{
    return (uint16_t)(adc_values[1] & 0x0FFFu);
}

/* 读取外部ADC原始值(12位, 0~4095)
 * channel: 0=PC2(外部输入1), 1=PC3(外部输入2) */
uint16_t MotorControl_GetExternalADC(uint8_t channel)
{
    if (channel == 0) {
        return (uint16_t)(adc_values[2] & 0x0FFFu);
    } else if (channel == 1) {
        return (uint16_t)(adc_values[3] & 0x0FFFu);
    }
    return 0;
}

/* 电流环PID: 返回PWM输出(-1000~+1000)
 * 由MotorControl_TimerTick100us每100μs调用一次 */
static int16_t MotorControl_CurrentLoop(MotorControl_t *mc)
{
    /* 电机未运行: 清积分, 输出0 */
    if (!motor_running) {
        mc->current_integral = 0.0f;
        mc->current_prev_error = 0.0f;
        return 0;
    }

    /* 过流保护已触发: 持续关闭输出, 等待用户复位 */
    if (mc->over_current_tripped) {
        mc->current_integral = 0.0f;
        return 0;
    }

    /* 电流环关闭: current_target直接作为PWM输出(向后兼容) */
    if (!mc->current_loop_en) {
        int16_t pwm = (int16_t)mc->current_target;
        if (pwm > mc->max_output) pwm = mc->max_output;
        if (pwm < -mc->max_output) pwm = -mc->max_output;
        return pwm;
    }

    /* ===== 电流环PID运算 ===== */
    const float dt = 0.0001f;  /* 100μs */

    float actual = MotorControl_ReadCurrent(mc);
    mc->current_actual = actual;

    /* 过流检测: 实际电流绝对值超过阈值时立即触发保护 */
    if (mc->over_current_limit > 0.0f && fabsf(actual) > mc->over_current_limit) {
        mc->over_current_tripped = 1;
        mc->current_integral = 0.0f;
        return 0;
    }

    float error = mc->current_target - actual;

    /* 积分累加 */
    mc->current_integral += error * dt;

    float P = mc->cur_Kp * error;
    float I = mc->cur_Ki * mc->current_integral;
    float D = 0.0f;
    if (mc->cur_Kd > 0.0f) {
        D = -mc->cur_Kd * (actual - mc->current_prev_error) / dt;
    }
    mc->current_prev_error = actual;

    int32_t output = (int32_t)(P + I + D);

    /* 限幅 + 抗积分饱和 */
    int16_t limit = mc->max_output;
    if (output > limit) {
        output = limit;
        mc->current_integral -= error * dt;
    }
    if (output < -limit) {
        output = -limit;
        mc->current_integral -= error * dt;
    }

    return (int16_t)output;
}

/* ========== 速度环（1ms周期, 新增）==========
 *
 * 仅在级联模式(POSITION_MODE/EXTERNAL_TARGET_MODE + max_run_speed>0)下运行.
 * 根据位置环输出的target_velocity与实际velocity做PID, 输出current_target.
 * 由MotorControl_TimerTick100us每10次(1ms)调用一次. */
static void MotorControl_SpeedLoop(MotorControl_t *mc)
{
    if (!motor_running || !mc->speed_loop_active) {
        mc->speed_loop_integral = 0.0f;
        mc->speed_loop_prev_vel = mc->velocity;
        return;
    }

    /* 过流/堵转已触发: 不做速度PID, current_target由上层清零 */
    if (mc->over_current_tripped || mc->stall_tripped) {
        mc->speed_loop_integral = 0.0f;
        mc->current_target = 0.0f;
        return;
    }

    const float dt = 0.001f;  /* 1ms */
    int16_t output_limit = mc->max_output;
    if (homing_state != HOMING_IDLE) {
        output_limit = mc->home_current;
        if (output_limit < 10 || output_limit > 1000) output_limit = 300;
    }

    float speed_error = mc->target_velocity - mc->velocity;

    mc->speed_loop_integral += speed_error * dt;

    float P = mc->spd_Kp * speed_error;
    float I = mc->spd_Ki * mc->speed_loop_integral;
    float D = -mc->spd_Kd * (mc->velocity - mc->speed_loop_prev_vel) / dt;
    mc->speed_loop_prev_vel = mc->velocity;

    int32_t output = (int32_t)(P + I + D);

    /* 限幅 + 抗积分饱和 */
    if (output > output_limit) {
        output = output_limit;
        mc->speed_loop_integral -= speed_error * dt;
    }
    if (output < -output_limit) {
        output = -output_limit;
        mc->speed_loop_integral -= speed_error * dt;
    }

    /* 限位开关钳制 */
    output = MotorControl_ApplyLimitSwitches(mc, output);

    /* 输出电流目标 */
    mc->current_target = (float)output;
}

/* 统一输出函数: 根据电流环是否使能, 把位置/速度环的output路由到current_target或直接PWM
 * - 电流环使能: output作为电流目标(current_target), 电流环100μs内根据它算PWM
 * - 电流环关闭: output直接作为PWM输出(保持原行为) */
static void MotorControl_SetOutput(MotorControl_t *mc, int16_t output)
{
    if (mc->current_loop_en) {
        mc->current_target = (float)output;
    } else {
        Motor_SetPWM(output);
    }
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
        mc->current_target = 0.0f;
        MotorControl_SetOutput(mc, 0);
        return 0;
    }

    /* ========== 停止功能（PIN_FUNC_STOP）==========
     * 按钮按下时强制PWM输出为0, 编码器继续计数, PID状态保留。
     * 松开后stop_override=0, PID从当前位置/速度恢复运行。 */
    if (stop_override) {
        ramped_target_speed = 0.0f;
        mc->current_target = 0.0f;
        MotorControl_SetOutput(mc, 0);
        return 0;
    }

    /* ========== 待机模式 ==========
     * 关闭PWM输出, 但编码器计数和速度计算已在上方正常执行,
     * 上位机仍可读取当前位置/速度。不进行堵转检测和任何模式PID。 */
    if (mc->mode == STANDBY_MODE) {
        ramped_target_speed = 0.0f;
        mc->current_target = 0.0f;
        MotorControl_SetOutput(mc, 0);
        return 0;
    }

    /* ========== 堵转保护 ==========
     * 检测条件: 电机"应该动但没动", 且持续 stall_time_ticks 个PID周期(5ms/tick)
     *   - 位置模式: 位置误差 > stall_err_limit (应该动) AND 实际速度近0 (没动)
     *   - 速度模式: 目标速度 > stall_err_limit (应该动) AND 实际速度近0 (没动)
     * 注意1: 仅"误差大"不能判定堵转 —— 长距离定位时误差天然很大但电机在正常转动
     * 注意2: 复位(homing)进行中不检测, 避免与homing自带的堵转检测冲突
     * "速度近0"阈值 = stall_err_limit/10 (最小5脉冲/秒), 由误差阈值派生, 无需额外配置 */
    if (mc->stall_protect_en && homing_state == HOMING_IDLE) {
        if (mc->stall_tripped) {
            /* 已触发: 持续关闭输出, 等待用户复位 */
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
            return 0;
        }
        if (mc->stall_err_limit > 0) {
            /* 速度近0阈值: 误差阈值的1/10, 最小5脉冲/秒 */
            float zero_speed_thresh = (float)mc->stall_err_limit / 10.0f;
            if (zero_speed_thresh < 5.0f) zero_speed_thresh = 5.0f;
            float abs_vel = fabsf(mc->velocity);
            uint8_t should_move_but_stopped = 0;  /* 1=应该动但没动(堵转特征) */

            if (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE) {
                /* 速度模式: 目标速度明显非0但电机不转 */
                float abs_target = fabsf(mc->target_speed);
                if (abs_target > (float)mc->stall_err_limit && abs_vel < zero_speed_thresh) {
                    should_move_but_stopped = 1;
                }
            } else {
                /* 位置模式: 位置误差大但电机不转 */
                int64_t pos_err = abs64(mc->target_position - current_position);
                if ((float)pos_err > (float)mc->stall_err_limit && abs_vel < zero_speed_thresh) {
                    should_move_but_stopped = 1;
                }
            }

            if (should_move_but_stopped) {
                mc->stall_tick_cnt++;
                if (mc->stall_tick_cnt >= mc->stall_time_ticks) {
                    mc->stall_tripped = 1;
                    mc->integral = 0;
                    mc->current_target = 0.0f;
                    MotorControl_SetOutput(mc, 0);
                    return 0;
                }
            } else {
                mc->stall_tick_cnt = 0;
            }
        }
    }

    if (homing_state == HOMING_SEARCH) {
        homing_ticks++;

        /* 原点位置开关检测: 如果配置了HOME功能的引脚有效，立即完成复位 */
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
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
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
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
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
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
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
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
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
                mc->current_target = 0.0f;
                MotorControl_SetOutput(mc, 0);
                return 0;
            }
            mc->integral = 0;
            mc->prev_error = 0;
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
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

    /* 外部目标位置模式：PB4优先于PB5；都无效时回到原点位置
     * 仅识别PIN_FUNC_TARGET_POS, 与外部目标速度模式PIN_FUNC_TARGET_SPEED分离, 防止错乱 */
    if (mc->mode == EXTERNAL_TARGET_MODE &&
        (mc->pin4_func == PIN_FUNC_TARGET_POS || mc->pin5_func == PIN_FUNC_TARGET_POS)) {
        uint8_t pin4 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t pin5 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
        if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) pin4 = !pin4;
        if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) pin5 = !pin5;

        uint8_t input = 0;
        int64_t target = 0;
        if (mc->pin4_func == PIN_FUNC_TARGET_POS && pin4) {
            input = 4;
            target = mc->pin4_target_pos;
        } else if (mc->pin5_func == PIN_FUNC_TARGET_POS && pin5) {
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

    /* 外部目标速度模式：PB4优先于PB5；都无效时停止
     * 仅识别PIN_FUNC_TARGET_SPEED, 与外部目标位置模式PIN_FUNC_TARGET_POS分离, 防止错乱 */
    if (mc->mode == EXTERNAL_TARGET_SPEED_MODE &&
        (mc->pin4_func == PIN_FUNC_TARGET_SPEED || mc->pin5_func == PIN_FUNC_TARGET_SPEED)) {
        uint8_t pin4 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t pin5 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
        if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) pin4 = !pin4;
        if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) pin5 = !pin5;

        uint8_t input = 0;
        float target = 0.0f;
        if (mc->pin4_func == PIN_FUNC_TARGET_SPEED && pin4) {
            input = 4;
            target = (float)mc->pin4_target_speed;
        } else if (mc->pin5_func == PIN_FUNC_TARGET_SPEED && pin5) {
            input = 5;
            target = (float)mc->pin5_target_speed;
        }

        if (target_level_state != input || mc->target_speed != target) {
            target_level_state = input;
            MotorControl_SetSpeed(mc, target);
        }
    }

    /* 开环模式：目标速度直接映射到输出，受output_limit和最大运行速度限制 */
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
        MotorControl_SetOutput(mc, (int16_t)output);
        return (int16_t)output;
    }

    /* ADC模式处理: PC2优先于PC3, 选中的引脚决定当前有效ADC值
     * ADC值归一化到0.0~1.0 (12位ADC原始值/4095.0)
     * 四种ADC模式分别对应:
     *   ADC_SPEED_MODE         - min_speed + ADC值×(max_speed - min_speed), 走速度PID
     *   ADC_POSITION_SPEED_MODE- min_speed + ADC值×(max_speed - min_speed), 走位置速度PID
     *   ADC_OPENLOOP_MODE      - min_pwm + ADC值×(max_pwm - min_pwm), 直接输出PWM
     *   ADC_POSITION_MODE      - min_position + ADC值×(max_position - min_position), 实时给目标位置走位置PID */
    if (mc->mode == ADC_SPEED_MODE || mc->mode == ADC_POSITION_SPEED_MODE ||
        mc->mode == ADC_OPENLOOP_MODE || mc->mode == ADC_POSITION_MODE) {
        /* 选择有效ADC通道: PC2优先, 仅当PC2=ADC_FUNC_NONE时才看PC3 */
        uint8_t active_func = mc->pc2_func;
        uint16_t adc_raw = MotorControl_GetExternalADC(0);  /* PC2 */
        if (mc->pc2_func == ADC_FUNC_NONE) {
            active_func = mc->pc3_func;
            adc_raw = MotorControl_GetExternalADC(1);  /* PC3 */
        }
        float adc_norm = (float)adc_raw / 4095.0f;
        if (adc_norm > 1.0f) adc_norm = 1.0f;
        if (adc_norm < 0.0f) adc_norm = 0.0f;

        /* ADC死区处理: 两个独立死区, ADC值落入任一死区时强制输出该位置对应的归一化值
         * 死区位置: 0=最小点(adc_norm=0), 1=中位点(adc_norm=0.5), 2=最大点(adc_norm=1.0)
         * 死区宽度: ADC原始值计数, 以选定位置为中心, 两侧各扩展width/2
         * 死区1和死区2独立设置, 可同时生效 */
        if (mc->adc_dead_zone1_width > 0 || mc->adc_dead_zone2_width > 0) {
            /* 死区1检查 */
            if (mc->adc_dead_zone1_width > 0) {
                uint16_t dz_center;
                float dz_norm;
                if (mc->adc_dead_zone1_pos == 0) {       /* 最小点 */
                    dz_center = 0;
                    dz_norm = 0.0f;
                } else if (mc->adc_dead_zone1_pos == 1) { /* 中位点 */
                    dz_center = 2048;
                    dz_norm = 0.5f;
                } else {                                  /* 最大点 */
                    dz_center = 4095;
                    dz_norm = 1.0f;
                }
                uint16_t dz_half = mc->adc_dead_zone1_width / 2;
                uint16_t dz_lower = (dz_center > dz_half) ? (dz_center - dz_half) : 0;
                uint16_t dz_upper = (dz_center < (4095 - dz_half)) ? (dz_center + dz_half) : 4095;
                if (adc_raw >= dz_lower && adc_raw <= dz_upper) {
                    adc_norm = dz_norm;
                }
            }
            /* 死区2检查 */
            if (mc->adc_dead_zone2_width > 0) {
                uint16_t dz_center;
                float dz_norm;
                if (mc->adc_dead_zone2_pos == 0) {       /* 最小点 */
                    dz_center = 0;
                    dz_norm = 0.0f;
                } else if (mc->adc_dead_zone2_pos == 1) { /* 中位点 */
                    dz_center = 2048;
                    dz_norm = 0.5f;
                } else {                                  /* 最大点 */
                    dz_center = 4095;
                    dz_norm = 1.0f;
                }
                uint16_t dz_half = mc->adc_dead_zone2_width / 2;
                uint16_t dz_lower = (dz_center > dz_half) ? (dz_center - dz_half) : 0;
                uint16_t dz_upper = (dz_center < (4095 - dz_half)) ? (dz_center + dz_half) : 4095;
                if (adc_raw >= dz_lower && adc_raw <= dz_upper) {
                    adc_norm = dz_norm;
                }
            }
        }

        if (mc->mode == ADC_SPEED_MODE || mc->mode == ADC_POSITION_SPEED_MODE) {
            /* 转速类: min_speed + ADC值×(max_speed - min_speed), 走速度或位置速度PID */
            float target = (float)mc->adc_min_speed + adc_norm * (float)(mc->adc_max_speed - mc->adc_min_speed);
            if (mc->target_speed != target) {
                MotorControl_SetSpeed(mc, target);
            }
        } else if (mc->mode == ADC_OPENLOOP_MODE) {
            /* 开环类: min_pwm + ADC值×(max_pwm - min_pwm), 直接输出(受限位和最大速度约束) */
            int32_t output = (int32_t)((float)mc->adc_min_pwm + adc_norm * (float)(mc->adc_max_pwm - mc->adc_min_pwm));
            if (output > output_limit) output = output_limit;
            if (output < -output_limit) output = -output_limit;
            if (mc->max_run_speed > 0 &&
                ((mc->velocity >= (float)mc->max_run_speed && output > 0) ||
                 (mc->velocity <= -(float)mc->max_run_speed && output < 0))) {
                output = 0;
            }
            output = MotorControl_ApplyLimitSwitches(mc, output);
            MotorControl_SetOutput(mc, (int16_t)output);
            return (int16_t)output;
        } else if (mc->mode == ADC_POSITION_MODE) {
            /* 位置类: min_position + ADC值×(max_position - min_position), 实时给目标位置走位置PID */
            int64_t target = mc->adc_min_position + (int64_t)(adc_norm * (float)(mc->adc_max_position - mc->adc_min_position));
            if (mc->target_position != target) {
                MotorControl_SetTarget(mc, target);
                mc->target_pos_frac = 0.0f;
                MODBUS_SyncTargetPosition(mc->target_position);
            }
        }
    }

    /* 脉冲累加：将EXTI中断中检测到的脉冲边缘累加到目标位置 */
    if (pulse_accumulator != 0) {
        __disable_irq();
        int32_t pulses = pulse_accumulator;
        pulse_accumulator = 0;
        __enable_irq();
        mc->target_position += pulses;
        MODBUS_SyncTargetPosition(mc->target_position);
    }

    /* 基于位置的速度模式：目标位置按速度累加（浮点累加避免低速截断） */
    if (mc->mode == VELOCITY_POSITION_MODE || mc->mode == ADC_POSITION_SPEED_MODE) {
        float limited_speed = mc->target_speed;
        if (mc->max_run_speed > 0) limited_speed = clampf(limited_speed, -(float)mc->max_run_speed, (float)mc->max_run_speed);
        mc->target_pos_frac += limited_speed * dt;
        int64_t inc = (int64_t)mc->target_pos_frac;
        mc->target_position += inc;
        mc->target_pos_frac -= (float)inc;
        MODBUS_SyncTargetPosition(mc->target_position);
    }

    float error;
    if (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE ||
        mc->mode == ADC_SPEED_MODE) {
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
            mc->speed_loop_active = 0;  /* 位置死区内停用速度环 */
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
            return 0;
        }
    }

    /* ========== 级联模式: 位置P环输出target_velocity, 速度PID由1ms速度环处理 ==========
     * 原5ms级联速度环已拆出至MotorControl_SpeedLoop(1ms周期),
     * 位置环只做位置P输出target_velocity, 速度环激活标志置1.
     * 速度环在TimerTick中每10次(1ms)调用一次, 根据target_velocity做速度PID输出current_target. */
    if ((mc->mode == POSITION_MODE || mc->mode == EXTERNAL_TARGET_MODE) && mc->max_run_speed > 0) {
        float target_vel = mc->pos_Kp * error;
        target_vel = clampf(target_vel, -(float)mc->max_run_speed, (float)mc->max_run_speed);
        mc->target_velocity = target_vel;
        mc->speed_loop_active = 1;  /* 激活1ms速度环 */
        mc->prev_error = error;
        mc->prev_velocity = mc->velocity;
        /* 调试信息: 记录位置环输出供上位机观察 */
        mc->pid_error = error;
        mc->pid_p = mc->pos_Kp * error;
        mc->pid_i = 0.0f;
        mc->pid_d = 0.0f;
        /* 电流目标由速度环(1ms)写入current_target, 此处不直接输出 */
        return 0;
    }

    float Kp = (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE ||
                 mc->mode == ADC_SPEED_MODE) ? mc->spd_Kp : mc->pos_Kp;
    float Ki = (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE ||
                 mc->mode == ADC_SPEED_MODE) ? mc->spd_Ki : mc->pos_Ki;
    float Kd = (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE ||
                 mc->mode == ADC_SPEED_MODE) ? mc->spd_Kd : mc->pos_Kd;

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
    mc->speed_loop_active = 0;  /* 单环PID模式不使用独立速度环 */
    MotorControl_SetOutput(mc, (int16_t)output);
    return (int16_t)output;
}

void MotorControl_Reset(MotorControl_t *mc)
{
    mc->integral = 0;
    mc->prev_error = 0;
    mc->last_tick = HAL_GetTick();
    mc->current_target = 0.0f;
    mc->current_integral = 0.0f;
    mc->speed_loop_active = 0;
    MotorControl_SetOutput(mc, 0);
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
    encoder_cnt_prev = 0;
    encoder_total_count = 0;

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* TIM3溢出中断已关闭: 编码器64位位置合成改在TIM1 100μs中断中
     * 通过差分累加实现(见Encoder_Synthesize), 不再依赖溢出中断修正wrap.
     * 这样彻底消除了"溢出已发生但中断未执行"窗口造成的累计误差. */
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    /* 启动ADC DMA连续转换(4通道: 电流+电压+2个外部ADC)
     * ADC1已配置为连续转换模式+DMA循环模式, 启动后DMA按Rank顺序把4个通道的
     * 转换结果循环搬到adc_values[4]数组中, 各功能模块直接读数组对应元素即可
     * 注意: HAL_ADC_Start_DMA内部会调用HAL_DMA_Start_IT使能传输完成中断,
     * 但ADC连续转换每1us就会触发一次中断, 严重干扰TIM3编码器(0,0)和TIM1 PID(2,0)
     * 所以启动后立即关闭DMA1_Channel1的NVIC使能, 让DMA在后台静默传输 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4) != HAL_OK) {
        /* ADC启动失败: 不阻塞运行, 电流环读到的值会是0(相当于无电流)
         * 电流环如果未使能(默认), 完全不影响原有功能 */
    }
    /* 关闭DMA1_Channel1中断, 让ADC DMA在后台静默传输 */
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}

void Motor_Stop(void)
{
    motor_running = 0;
    Motor_SetPWM(0);
    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
    /* 停止ADC DMA转换(电流采样) */
    HAL_ADC_Stop_DMA(&hadc1);
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
static void Encoder_Synthesize(void)
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

/* TIM1 100us滴答, 三环分频控制:
 *   每 1 次  (100μs): 电流环 PID   - 读ADC电流, 根据current_target做PID输出PWM
 *   每10次  (1ms):   速度环 PID   - 级联模式下根据target_velocity做PID输出current_target
 *   每50次  (5ms):   位置环 PID   - MotorControl_Update计算位置误差, 输出target_velocity或current_target
 *
 * 数据流(级联模式 POSITION_MODE + max_run_speed>0):
 *   位置环(5ms) → target_velocity → 速度环(1ms) → current_target → 电流环(100μs) → PWM
 *
 * 数据流(单环模式 SPEED_MODE / POSITION_MODE无max_run_speed):
 *   位置/速度环(5ms) → current_target → 电流环(100μs) → PWM
 */
void MotorControl_TimerTick100us(void)
{
    static uint16_t speed_div = 0;    /* 速度环分频计数器(1ms) */
    static uint16_t pid_divider = 0;  /* 位置环分频计数器(5ms) */

    /* ===== 编码器位置合成: 每100μs读一次TIM3 CNT做差分累加 =====
     * 必须最先执行, 保证后续转速采集/位置环用的都是最新位置.
     * 差分用int16_t补码自动处理65536模回绕, 不依赖任何中断. */
    Encoder_Synthesize();

    /* 转速采集：按分频值采样到1KB缓冲区 */
    if (motor_ctl && motor_ctl->speed_acq_active && !motor_ctl->speed_acq_done) {
        if (motor_ctl->speed_acq_div_cnt == 0) {
            if (motor_ctl->speed_acq_count < SPEED_ACQ_BUF_SIZE) {
                int16_t sample;
                if (motor_ctl->speed_acq_type == 1) {
                    /* 采集PWM输出 (-1000~+1000) */
                    sample = current_pwm_output;
                } else if (motor_ctl->speed_acq_type == 2) {
                    /* 采集位置(相对起始位置的偏移, int16_t范围±32767脉冲) */
                    int64_t diff = Encoder_GetCount() - motor_ctl->speed_acq_pos_start;
                    if (diff > 32767) diff = 32767;
                    if (diff < -32767) diff = -32767;
                    sample = (int16_t)diff;
                } else if (motor_ctl->speed_acq_type == 3) {
                    /* 采集电流(相对值, ±1000对应±8.25A, int16_t范围±32767)
                     * 直接读ADC, 不依赖电流环是否使能 */
                    float cur = MotorControl_ReadCurrent(motor_ctl);
                    motor_ctl->current_actual = cur;  /* 顺便刷新实际电流值 */
                    if (cur > 32767.0f) cur = 32767.0f;
                    if (cur < -32767.0f) cur = -32767.0f;
                    sample = (int16_t)cur;
                } else if (motor_ctl->speed_acq_type == 4) {
                    /* 采集PC0电压ADC原始值 (0~4095, 12位ADC, 分压100K:10K检测供电电压)
                     * ADC值0~4095完全在int16_t正数范围内, 无符号问题 */
                    sample = (int16_t)MotorControl_GetSupplyVoltageADC();
                } else if (motor_ctl->speed_acq_type == 5) {
                    /* 采集PC2外部ADC原始值 (0~4095, 12位ADC, 电位器等外部输入) */
                    sample = (int16_t)MotorControl_GetExternalADC(0);
                } else if (motor_ctl->speed_acq_type == 6) {
                    /* 采集PC3外部ADC原始值 (0~4095, 12位ADC, 电位器等外部输入) */
                    sample = (int16_t)MotorControl_GetExternalADC(1);
                } else {
                    /* 采集转速 (脉冲/秒, int16_t范围±32767) */
                    float spd = Encoder_GetSpeed();
                    if (spd > 32767.0f) spd = 32767.0f;
                    if (spd < -32767.0f) spd = -32767.0f;
                    sample = (int16_t)spd;
                }
                motor_ctl->speed_acq_buffer[motor_ctl->speed_acq_count] = sample;
                motor_ctl->speed_acq_count++;
                if (motor_ctl->speed_acq_count >= motor_ctl->speed_acq_size) {
                    motor_ctl->speed_acq_done = 1;
                    motor_ctl->speed_acq_active = 0;
                }
            }
        }
        motor_ctl->speed_acq_div_cnt++;
        if (motor_ctl->speed_acq_div_cnt >= motor_ctl->speed_acq_divider) {
            motor_ctl->speed_acq_div_cnt = 0;
        }
    }

    /* ===== 电流环: 每1次(100μs)执行 =====
     * 电流环仅当使能时才运行并输出PWM:
     *   - 使能时: 读ADC→电流PID→输出PWM
     *   - 未使能时: 不做任何事, PWM由位置/速度环通过MotorControl_SetOutput直接控制
     *     (避免电流环用stale的current_target覆盖位置/速度环刚设置的PWM)
     * 待机模式(STANDBY_MODE)下跳过电流环, 确保PWM保持为0 */
    if (motor_ctl && motor_ctl->current_loop_en && motor_ctl->mode != STANDBY_MODE) {
        int16_t pwm = MotorControl_CurrentLoop(motor_ctl);
        Motor_SetPWM(pwm);
    }

    /* ===== 速度环: 每10次(1ms)执行 =====
     * 仅级联模式(POSITION_MODE+max_run_speed>0)下激活,
     * 根据位置环输出的target_velocity做速度PID, 输出current_target */
    if (++speed_div >= 10) {
        speed_div = 0;
        if (motor_ctl) {
            MotorControl_SpeedLoop(motor_ctl);
        }
    }

    /* ===== 位置环: 每50次(5ms)执行 =====
     * 计算位置误差, 级联模式输出target_velocity, 单环模式输出current_target */
    pid_divider++;
    if (pid_divider >= 50) {
        pid_divider = 0;
        if (motor_ctl) {
            MotorControl_Update(motor_ctl, Encoder_GetCount());
        }
    }
}

/* ========== 堵转保护接口 ========== */

uint8_t MotorControl_IsStallTripped(void)
{
    return motor_ctl ? motor_ctl->stall_tripped : 0;
}

void MotorControl_ResetStall(MotorControl_t *mc)
{
    /* 关中断清零, 避免与PID中断竞争 */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->stall_tripped = 0;
    mc->stall_tick_cnt = 0;
    mc->integral = 0;
    mc->prev_error = 0;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

/* ========== 电流环接口实现（新增）========== */

void MotorControl_SetCurPID(MotorControl_t *mc, float Kp, float Ki, float Kd)
{
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->cur_Kp = Kp;
    mc->cur_Ki = Ki;
    mc->cur_Kd = Kd;
    mc->current_integral = 0.0f;
    mc->current_prev_error = 0.0f;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void MotorControl_EnableCurrentLoop(MotorControl_t *mc, uint8_t en)
{
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->current_loop_en = en ? 1 : 0;
    /* 切换电流环使能状态时清零积分, 防止残留积分导致输出突变 */
    mc->current_integral = 0.0f;
    mc->current_prev_error = 0.0f;
    mc->current_target = 0.0f;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void MotorControl_SetCurrentCalib(MotorControl_t *mc, uint16_t offset, float scale)
{
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->current_offset = offset;
    mc->current_scale = scale;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void MotorControl_SetOverCurrentLimit(MotorControl_t *mc, float limit)
{
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->over_current_limit = limit;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

uint8_t MotorControl_IsOverCurrentTripped(void)
{
    return motor_ctl ? motor_ctl->over_current_tripped : 0;
}

void MotorControl_ResetOverCurrent(MotorControl_t *mc)
{
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->over_current_tripped = 0;
    mc->current_integral = 0.0f;
    mc->current_prev_error = 0.0f;
    mc->current_target = 0.0f;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

float MotorControl_GetCurrent(void)
{
    return motor_ctl ? motor_ctl->current_actual : 0.0f;
}

float MotorControl_GetCurrentTarget(void)
{
    return motor_ctl ? motor_ctl->current_target : 0.0f;
}
