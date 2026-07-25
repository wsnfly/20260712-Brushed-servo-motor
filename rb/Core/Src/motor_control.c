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
#include "input.h"   /* Input_Init + 共享状态 pulse_accumulator/stop_override */
#include "ajiajiansu.h"  /* 步进加减速: bujindianji[]/StepAccel_Restart() */
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

extern void MODBUS_SyncTargetPosition(int64_t position);
/* MODBUS_TriggerStartFlag 已随PB4/PB5输入处理迁至 input.c */

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
    /* 外部ADC滤波窗口大小(新增, 加在末尾) */
    uint32_t adc_filter_size;
    /* 堵转保护参数 */
    uint32_t stall_protect_en;
    uint32_t stall_err_limit;
    uint32_t stall_time_ticks;
    /* 电流限幅参数(新增, 加在末尾不破坏已有Flash偏移) */
    uint32_t max_current;
    /* 电流死区参数(新增, 加在末尾)
     * 实际电流绝对值<此值时强制PWM=0, 防止静止时细微电流 */
    uint32_t current_dead_zone;
    /* 电流环使能时的速度环/位置环PID参数(双套PID自动切换)
     * 电流环改变被控对象特性(电压控制→电流控制), 需要不同PID参数.
     * 电流环关闭时使用 spd_Kp/Ki/Kd, pos_Kp/Ki/Kd
     * 电流环使能时使用 spd_Kp_cur/Ki_cur/Kd_cur, pos_Kp_cur/Ki_cur/Kd_cur */
    uint32_t spd_kp_cur;
    uint32_t spd_ki_cur;
    uint32_t spd_kd_cur;
    uint32_t pos_kp_cur;
    uint32_t pos_ki_cur;
    uint32_t pos_kd_cur;
    /* 步进加减速使能(新增, 加在末尾不破坏已有Flash偏移) */
    uint32_t step_accel_en;
    /* 步进最大速度索引(新增): zuidasudu是int64_t, 拆为高/低32位存储.
     * 断电后zuidasudu默认为0, ajiajiansu.c会自动设为63(对应6400步/秒),
     * 导致用户设置的最大速度丢失. 保存到Flash后开机恢复用户设置值. */
    uint32_t step_max_speed_h;  /* int64高32位 */
    uint32_t step_max_speed_l;  /* int64低32位 */
    /* 电流环前馈系数(新增, 加在末尾不破坏已有Flash偏移) */
    uint32_t cur_kff;
    /* 位置环速度前馈系数(新增): 步进加减速模式消除积分windup抖动 */
    uint32_t pos_kff;
} FlashConfig_t;

typedef enum {
    HOMING_IDLE = 0,
    HOMING_SEARCH,
    HOMING_BACK,
    HOMING_SWITCH_SEARCH,              /* 开关复位：向负方向搜索开关 */
    HOMING_SWITCH_RELEASE,             /* 开关复位：向正方向离开开关 */
    HOMING_SWITCH_PRECISION_APPROACH,  /* 开关复位：低速接近开关 */
    HOMING_SWITCH_PRECISION_RELEASE,   /* 开关复位：低速离开开关 */
    HOMING_SWITCH_BACK,                /* 开关复位：退回偏置距离后设为零点 */
    HOMING_BIDIR_NEG_SEARCH,           /* 双向复位：负方向搜索(堵转/限位传感器) */
    HOMING_BIDIR_POS_SEARCH,           /* 双向复位：正方向搜索(堵转/限位传感器) */
    HOMING_BIDIR_CENTER                /* 双向复位：运动到中点+偏置位置后设为零点 */
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
        if (cfg->home_mode <= 4) {
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
        /* 加载电流限幅(新增): NaN检查+范围检查, 无效时保留默认值0(不限制) */
        f = u32_to_float(cfg->max_current);
        if (f == f && f >= 0.0f) mc->max_current = f;
        /* 加载电流死区(新增): NaN检查+范围检查, 无效时保留默认值0(关闭) */
        f = u32_to_float(cfg->current_dead_zone);
        if (f == f && f >= 0.0f) mc->current_dead_zone = f;
        /* 加载电流环使能时的PID参数(双套PID): 无效时默认为电压模式参数的1/3 */
        f = u32_to_float(cfg->spd_kp_cur);
        if (f == f && f > 0.0f) mc->spd_Kp_cur = f;
        f = u32_to_float(cfg->spd_ki_cur);
        if (f == f && f >= 0.0f) mc->spd_Ki_cur = f;
        f = u32_to_float(cfg->spd_kd_cur);
        if (f == f && f >= 0.0f) mc->spd_Kd_cur = f;
        f = u32_to_float(cfg->pos_kp_cur);
        if (f == f && f > 0.0f) mc->pos_Kp_cur = f;
        f = u32_to_float(cfg->pos_ki_cur);
        if (f == f && f >= 0.0f) mc->pos_Ki_cur = f;
        f = u32_to_float(cfg->pos_kd_cur);
        if (f == f && f >= 0.0f) mc->pos_Kd_cur = f;
        /* 加载电流环前馈系数(新增): NaN检查+范围检查, 无效时默认0(关闭) */
        f = u32_to_float(cfg->cur_kff);
        if (f == f && f >= 0.0f) mc->cur_Kff = f;
        /* 加载位置环速度前馈系数(新增): NaN检查, 无效时默认0(关闭) */
        f = u32_to_float(cfg->pos_kff);
        if (f == f && f >= 0.0f) mc->pos_Kff = f;

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
        /* 加载外部ADC滤波窗口(新增): 范围0~128, 超出视为无效保留默认0 */
        if (cfg->adc_filter_size <= 128) {
            mc->adc_filter_size = (uint8_t)cfg->adc_filter_size;
        }
        /* 加载堵转保护参数 */
        mc->stall_protect_en = (cfg->stall_protect_en != 0) ? 1 : 0;
        mc->stall_err_limit = (int32_t)cfg->stall_err_limit;
        mc->stall_time_ticks = (uint16_t)cfg->stall_time_ticks;
        /* 加载步进加减速使能(新增) */
        mc->step_accel_en = (cfg->step_accel_en != 0) ? 1 : 0;
        /* 加载步进最大速度索引(新增): 从Flash恢复zuidasudu, 避免断电后变为默认值63.
         * 注意: 此时bujindianji[0]已经初始化, 直接写入;
         * 如果Flash中值为0(未保存过), ajiajiansu.c的默认检查会设为63 */
        if (cfg->step_max_speed_h != 0xFFFFFFFF || cfg->step_max_speed_l != 0xFFFFFFFF) {
            int64_t saved_speed = ((int64_t)cfg->step_max_speed_h << 32) | cfg->step_max_speed_l;
            if (saved_speed > 0) {
                __disable_irq();
                bujindianji[0].zuidasudu = saved_speed;
                __enable_irq();
            }
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
    /* 保存外部ADC滤波窗口(新增)
     * 注意: 偏移必须与FlashConfig_t结构体字段顺序严格一致!
     * 之前错写到280, 导致与结构体偏移236不匹配, LoadConfig通过字段名访问时
     * 读到的是stall_protect_en的值, 后续所有字段全部错位4字节.
     * 受影响字段: adc_filter_size/stall_max_current/current_dead_zone/双套PID
     * 这正是"电流环设置断电不保存"问题的根因: 保存到错位, 加载读到错误值.
     * 修正: 按结构体顺序, adc_filter_size在偏移236, stall_protect_en在240, 以此类推 */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 236, (uint32_t)mc->adc_filter_size);
    /* 保存堵转保护参数 */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 240, (uint32_t)mc->stall_protect_en);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 244, (uint32_t)mc->stall_err_limit);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 248, (uint32_t)mc->stall_time_ticks);
    /* 保存电流限幅参数(新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 252, float_to_u32(mc->max_current));
    /* 保存电流死区参数(新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 256, float_to_u32(mc->current_dead_zone));
    /* 保存电流环使能时的PID参数(双套PID, 新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 260, float_to_u32(mc->spd_Kp_cur));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 264, float_to_u32(mc->spd_Ki_cur));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 268, float_to_u32(mc->spd_Kd_cur));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 272, float_to_u32(mc->pos_Kp_cur));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 276, float_to_u32(mc->pos_Ki_cur));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 280, float_to_u32(mc->pos_Kd_cur));
    /* 保存步进加减速使能(新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 284, (uint32_t)mc->step_accel_en);
    /* 保存步进最大速度索引(新增): int64拆为高/低32位 */
    {
        int64_t zuidasudu;
        __disable_irq();
        zuidasudu = bujindianji[0].zuidasudu;
        __enable_irq();
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 288, (uint32_t)(zuidasudu >> 32));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 292, (uint32_t)(zuidasudu & 0xFFFFFFFF));
    }
    /* 保存电流环前馈系数(新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 296, float_to_u32(mc->cur_Kff));
    /* 保存位置环速度前馈系数(新增) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 300, float_to_u32(mc->pos_Kff));

    HAL_FLASH_Lock();
}

static uint8_t motor_running = 0;
static float current_velocity = 0.0f;
static float ramped_target_speed = 0.0f;

/* ========== 64位编码器累计计数已拆分至 encoder 模块(encoder.c) ==========
 * 原理: TIM1 100μs中断里对TIM3->CNT做int16_t差分累加, 合成64位无歧义位置.
 * 方向反转/速度源通过 Encoder_BindDirection/Encoder_BindVelocity 绑定,
 * 由 MotorControl_Init 在加载配置后同步. */

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
/* ADC DMA采样缓冲区(4通道×25组, DMA循环模式持续刷新)
 * 数组元素按ADC规则通道Rank顺序循环对应:
 *   [0,4,8...] = Rank1 = ADC_CHANNEL_11 = PC1 (电流采样)
 *   [1,5,9...] = Rank2 = ADC_CHANNEL_10 = PC0 (供电电压检测, 100K:10K分压, ×11)
 *   [2,6,10...] = Rank3 = ADC_CHANNEL_12 = PC2 (外部ADC输入1, 电位器调速等)
 *   [3,7,11...] = Rank4 = ADC_CHANNEL_13 = PC3 (外部ADC输入2, 电位器调速等)
 * 使用uint16_t匹配DMA的HALFWORD传输配置, 每通道25个样本用于平均滤波 */
#define ADC_CHANNEL_COUNT 4u
#define ADC_AVG_SAMPLE_COUNT 25u
#define ADC_DMA_VALUE_COUNT (ADC_CHANNEL_COUNT * ADC_AVG_SAMPLE_COUNT)
static volatile uint16_t adc_values[ADC_DMA_VALUE_COUNT];

/* 外部ADC滑动平均滤波器状态(新增)
 * 2个独立通道(PC2=channel0, PC3=channel1), 各自维护一个环形缓冲区.
 * 当mc.adc_filter_size<=1时不启用滤波, 直接返回原始ADC值.
 * 缓冲区大小128对应最大窗口128, 主循环5ms周期下最大延迟约640ms,
 * 满足用户"最大延迟500ms"的需求(100点≈500ms, 留余量到128).
 * 内存占用: 2通道 × 128 × 2字节 = 512字节 SRAM */
#define ADC_FILTER_MAX_SIZE 128
static uint16_t adc_filter_buf[2][ADC_FILTER_MAX_SIZE];
static uint8_t  adc_filter_idx[2] = {0, 0};    /* 环形缓冲区写索引(0~127, uint8_t足够) */
static uint8_t  adc_filter_count[2] = {0, 0};  /* 已填充样本数(启动阶段<窗口大小) */
static uint32_t adc_filter_sum[2] = {0, 0};    /* 当前窗口累加和(避免每次重算) */
static uint8_t  adc_filter_prev_size[2] = {0, 0}; /* 上次窗口大小(检测size变化触发重置) */

#define SPEED_RAMP_ACCEL 10000.0f  /* 速度模式目标变化斜坡: 脉冲/秒^2 */

/* 开关精确复位专用变量 */
static int64_t switch_edge_pos = 0;       /* 开关边缘触发位置 */
static uint8_t precision_cycle_idx = 0;   /* 当前精确检测循环次数 */
static int64_t switch_edge_sum = 0;       /* 边缘位置累加（用于求平均） */

/* 双向复位专用变量 */
static int64_t bidir_neg_position = 0;    /* 负方向停止位置(堵转/限位触发点) */
static int64_t bidir_pos_position = 0;    /* 正方向停止位置(堵转/限位触发点) */

/* 外部目标电平模式状态(本模块 MotorControl_Update 内部使用, 保留于此) */
static uint8_t target_level_state = 0;

/* PB4/PB5输入处理(停止/复位/启动标志位/脉冲/方向)已拆分至 input 模块(input.c):
 *   - prev_pin4_state/prev_pin5_state, home_start_btn*, startflag_btn* 状态变量
 *   - HOME_START_COOLDOWN_MS 宏
 *   - pulse_accumulator / stop_override (在 input.h 中 extern 声明, 由 input.c 定义,
 *     本模块 MotorControl_Update 消费) */

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

static uint16_t MotorControl_ReadADCAverage(uint8_t channel)
{
    uint32_t sum = 0;

    if (channel >= ADC_CHANNEL_COUNT) {
        return 0;
    }

    for (uint8_t i = 0; i < ADC_AVG_SAMPLE_COUNT; i++) {
        sum += (uint32_t)(adc_values[channel + ((uint16_t)i * ADC_CHANNEL_COUNT)] & 0x0FFFu);
    }

    return (uint16_t)(sum / ADC_AVG_SAMPLE_COUNT);
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

/* 读取指定方向的限位开关状态（有效=1，无效=0）
 * dir: 0=正方向限位(limit_dir=0, 停止正方向), 1=负方向限位(limit_dir=1, 停止负方向)
 * 用于双向传感器居中复位检测正/负限位到达 */
static uint8_t ReadLimitSwitch(MotorControl_t *mc, uint8_t dir)
{
    uint8_t pin4 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t pin5 = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) ? 1 : 0;
    if (mc->pin4_polarity == POLARITY_ACTIVE_LOW) pin4 = !pin4;
    if (mc->pin5_polarity == POLARITY_ACTIVE_LOW) pin5 = !pin5;
    if (mc->pin4_func == PIN_FUNC_LIMIT && pin4 && mc->pin4_limit_dir == dir) return 1;
    if (mc->pin5_func == PIN_FUNC_LIMIT && pin5 && mc->pin5_limit_dir == dir) return 1;
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
    mc->adc_filter_size = 0;         /* 默认关闭ADC滤波(直接用原始值, 兼容旧固件) */
    mc->mode = POSITION_MODE;
    /* ===== 堵转保护配置默认值(需在LoadConfig前设置, 否则会被默认值覆盖Flash值) ===== */
    mc->stall_protect_en = 0;
    mc->stall_err_limit = 200;       /* 默认200脉冲 / 200脉冲/秒 */
    mc->stall_time_ticks = 200;      /* 默认200个tick = 1秒持续 */
    /* ===== 电流环配置默认值(需在LoadConfig前设置, 否则会被默认值覆盖Flash值) =====
     * 默认关闭电流环(current_loop_en=0), 行为与原代码完全一致.
     * 用户需配置电流采样硬件后, 通过MotorControl_EnableCurrentLoop开启.
     * 默认PID: Kp=15, Ki=0, Kd=0 (纯P控制)
     *   新算法(电流限幅不截顶target): target可达900, error=888, P=13320→PWM截顶900(满量程)
     *   电流超限时PWM按比例缩小: PWM *= max_current/actual
     *   Ki=0: PID输出已满量程(900), 无需积分消除静差. 纯P控制稳定无windup.
     * 默认标定: offset=2048(12位ADC中点), scale=1000/2048≈0.488
     *   即 ADC值 0→电流-1000, ADC值 2048→电流0, ADC值 4095→电流+1000
     * 默认过流保护: 关闭(over_current_limit=0)
     * 默认电流限幅: 不限制(max_current=0) */
    mc->cur_Kp = 15.0f;
    mc->cur_Ki = 0.0f;
    mc->cur_Kd = 0.0f;
    mc->cur_Kff = 0.0f;  /* 默认关闭前馈, 用户通过Modbus(0x009B)设置, 建议150~250(=0.015~0.025) */
    mc->pos_Kff = 0.0f;  /* 默认关闭位置环速度前馈, 用户通过Modbus(0x009C)设置 */
    mc->prev_target_position = 0;
    mc->current_offset = 2048;
    mc->current_scale = 1000.0f / 2048.0f;
    mc->current_loop_en = 0;
    mc->over_current_limit = 0.0f;
    mc->max_current = 0.0f;  /* 默认不限制电流 */
    mc->current_dead_zone = 0.0f;  /* 默认关闭电流死区(为兼容旧固件, 用户按需开启) */
    /* 电流环使能时的PID参数默认值(电压模式参数的1/3, 仅占位, 需用户实际调参)
     * LoadConfig时如Flash有有效值则覆盖 */
    mc->spd_Kp_cur = mc->spd_Kp / 3.0f;
    mc->spd_Ki_cur = mc->spd_Ki / 5.0f;
    mc->spd_Kd_cur = mc->spd_Kd;
    mc->pos_Kp_cur = mc->pos_Kp / 3.0f;
    mc->pos_Ki_cur = mc->pos_Ki / 5.0f;
    mc->pos_Kd_cur = mc->pos_Kd;
    /* 从Flash加载保存的配置，覆盖默认值（无有效数据时保留默认值） */
    MotorControl_LoadConfig(mc);
    /* 同步编码器方向/速度源到 encoder 模块(配置加载完成后方向已知) */
    Encoder_BindDirection(mc->encoder_dir == ENCODER_DIR_REVERSED ? 1 : 0);
    Encoder_BindVelocity(&current_velocity);
    /* 初始化输入模块: 绑定实例指针并根据极性初始化PB4/PB5边沿检测状态
     * (避免首次触发虚假脉冲) */
    Input_Init(mc);
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
    mc->speed_acq_channels = 1;  /* 默认单通道(兼容原行为) */
    mc->speed_acq_type2 = 0;     /* 通道2默认转速 */
    mc->speed_acq_type3 = 0;     /* 通道3默认转速 */
    mc->speed_acq_type4 = 0;     /* 通道4默认转速 */
    mc->speed_acq_size = SPEED_ACQ_BUF_SIZE;  /* 默认采满缓冲区 */

    /* 堵转保护运行时状态(配置参数已在LoadConfig前设置默认值并从Flash加载) */
    mc->stall_tick_cnt = 0;
    mc->stall_tripped = 0;

    /* ===== 电流环运行时状态初始化 =====
     * 配置参数(cur_Kp/Ki/Kd, offset, scale, current_loop_en, over_current_limit,
     * max_current)已在LoadConfig前设置默认值并从Flash加载, 此处仅清零运行时状态. */
    mc->current_target = 0.0f;
    mc->current_actual = 0.0f;
    mc->current_integral = 0.0f;
    mc->current_prev_error = 0.0f;
    mc->over_current_tripped = 0;

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
    mc->prev_target_position = 0;  /* 同步目标位置, 防止D项/FF计算出现速度尖峰 */
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
    /* 步进加减速: 复位完成后编码器已清零, 同步算法位置到0,
     * 防止yudingweizhi残留导致复位后电机突然转动 */
    if (mc->step_accel_en) {
        StepAccel_SyncPosition(0);
    }
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

    /* ========== 双向复位（堵转/传感器居中）==========
     * 流程: 负方向搜索→正方向搜索→计算中点→运动到中点+偏置→清零设原点
     * 堵转模式(BIDIR_STALL): 两端靠堵转检测停止
     * 传感器模式(BIDIR_SENSOR): 两端靠正/负限位开关停止
     * 启动逻辑两种模式相同, 区别在Update状态机中的停止条件检测 */
    if (mc->home_mode == HOME_MODE_BIDIR_STALL || mc->home_mode == HOME_MODE_BIDIR_SENSOR) {
        if (mc->home_current < 10 || mc->home_current > 1000) mc->home_current = 300;
        if (mc->home_speed <= 0) mc->home_speed = 1000;
        if (mc->home_max_distance <= 0) mc->home_max_distance = 10000;

        homing_state = HOMING_BIDIR_NEG_SEARCH;
        homing_failed = 0;
        homing_ticks = 0;
        homing_stall_ticks = 0;
        homing_start_position = Encoder_GetCount();
        mode_before_homing = mc->mode;
        bidir_neg_position = 0;
        bidir_pos_position = 0;

        mc->mode = VELOCITY_POSITION_MODE;
        mc->target_position = homing_start_position;
        mc->target_speed = -(float)mc->home_speed;  /* 先负方向运动 */
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
    if (mc->step_accel_en) {
        /* 步进加减速模式: 目标送入yudingweizhi, 算法生成平滑轨迹
         * 不能直接设target_position, 会被TIM1中断中的dangqianweizhi覆盖 */
        __disable_irq();
        bujindianji[0].yudingweizhi = target;
        __enable_irq();
        StepAccel_Restart();
    } else {
        /* 普通模式: 直接设目标位置, 清积分和误差 */
        mc->target_position = target;
        mc->integral = 0;
        mc->prev_error = 0;
    }
}

void MotorControl_SetSpeed(MotorControl_t *mc, float speed)
{
    /* 速度目标变化时清零积分, 防止积分饱和(windup)导致减速延迟.
     * 典型场景: 电机达不到目标速度时积分持续累积, 设速度为0后
     *   积分仍为大的正值, current_target保持正向, 电机继续运行数秒.
     * 清零积分后, 速度误差直接反映在P项, 电机能立即响应减速.
     * 仅在目标变化较大(>200脉冲/秒)或目标为0(停止)时清零,
     *   避免ADC/外部模式的小幅抖动导致稳态误差.
     * 关TIM1中断保证原子性(积分在100us中断中被读写). */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    float old_speed = mc->target_speed;
    mc->target_speed = speed;
    if (speed == 0.0f || fabsf(speed - old_speed) > 200.0f) {
        mc->integral = 0.0f;
        mc->speed_loop_integral = 0.0f;
    }
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void MotorControl_SetMode(MotorControl_t *mc, MotorMode_t mode)
{
    if (mode > ADC_POSITION_MODE) {
        return;
    }

    if (mc->mode != mode) {
        if (mode == POSITION_MODE || mode == VELOCITY_POSITION_MODE || mode == EXTERNAL_TARGET_MODE ||
            mode == ADC_POSITION_SPEED_MODE) {
            /* 切到位置模式: 目标位置=当前实际位置, 防止位置误差导致突然运行 */
            mc->target_position = Encoder_GetCount();
            mc->target_pos_frac = 0.0f;
            mc->target_speed = 0.0f;
            ramped_target_speed = 0.0f;  /* 清零速度斜坡, 确保电机停转 */
        } else if (mode == SPEED_MODE || mode == OPENLOOP_MODE || mode == EXTERNAL_TARGET_SPEED_MODE ||
                   mode == ADC_SPEED_MODE) {
            /* 切到速度模式: 目标速度清零, 电机停转 */
            mc->target_speed = 0.0f;
            ramped_target_speed = 0.0f;  /* 清零速度斜坡, 立即停转而非从当前速度缓慢减速 */
        } else if (mode == ADC_POSITION_MODE) {
            mc->target_position = 0;
            mc->target_pos_frac = 0.0f;
            ramped_target_speed = 0.0f;
        }
        /* 步进加减速: 模式切换时同步算法位置到当前编码器位置,
         * 防止yudingweizhi/dangqianweizhi残留旧值导致切换后目标位置跳变、电机突然转动.
         * 对所有模式切换都执行(包括切到非位置模式), 确保下次切回位置模式时算法位置是新鲜的 */
        if (mc->step_accel_en) {
            StepAccel_SyncPosition(Encoder_GetCount());
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
    /* 立即停止PWM输出, 确保模式切换瞬间电机停转
     * 电流环使能时: current_target=0, 电流环PID下次中断输出PWM=0
     * 电流环关闭时: 直接Motor_SetPWM(0) */
    MotorControl_SetOutput(mc, 0);
}

void MotorControl_SetDirection(MotorControl_t *mc, MotorDir_t dir)
{
    mc->direction = dir;
}

void MotorControl_SetEncoderDirection(MotorControl_t *mc, EncoderDir_t dir)
{
    mc->encoder_dir = dir;
    /* 同步到 encoder 模块, 使 Encoder_GetCount/GetSpeed 的方向取反立即生效 */
    Encoder_BindDirection(dir == ENCODER_DIR_REVERSED ? 1 : 0);
}

void MotorControl_SetPosPID(MotorControl_t *mc, float Kp, float Ki, float Kd)
{
    /* 关TIM1中断保护: PID参数在主循环被修改, TIM1中断读取使用.
     * 若不保护, 中断可能在Ki已更新但integral尚未清零时触发, 用新Ki×旧积分导致输出突变.
     * 与MotorControl_SetCurPID保持一致的并发保护策略. */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->pos_Kp = Kp;
    mc->pos_Ki = Ki;
    mc->pos_Kd = Kd;
    mc->integral = 0;
    mc->prev_error = 0;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void MotorControl_SetSpdPID(MotorControl_t *mc, float Kp, float Ki, float Kd)
{
    /* 关TIM1中断保护: 同SetPosPID, 防止PID参数与积分状态不一致导致输出突变 */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->spd_Kp = Kp;
    mc->spd_Ki = Ki;
    mc->spd_Kd = Kd;
    mc->integral = 0;
    mc->prev_error = 0;
    mc->speed_loop_integral = 0.0f;  /* 清零1ms速度环独立积分 */
    mc->speed_loop_prev_vel = mc->velocity;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
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

    /* 步进加减速: 设置零点时同步算法位置到0, 防止yudingweizhi残留导致切换后突然转动 */
    if (mc->step_accel_en) {
        StepAccel_SyncPosition(0);
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

/* ========== 电流环（100μs周期, 新增）==========
 *
 * 读取ADC电流采样值, 与current_target做PID, 输出PWM(-1000~+1000).
 * 电流环使能后由TIM1 100μs中断每次调用.
 * 电流环关闭时: 电流环函数返回current_target作为PWM(透传, 等效于原无电流环行为). */

/* 读取ADC电流采样值并转换为相对电流(-1000~+1000)
 * DMA循环模式下, adc_values由DMA持续刷新, 读取对应通道25个样本取平均 */
static float MotorControl_ReadCurrent(MotorControl_t *mc)
{
    uint16_t adc_val = MotorControl_ReadADCAverage(0);  /* 12位ADC, 屏蔽高4位 */
    return ((float)adc_val - (float)mc->current_offset) * mc->current_scale;
}

/* 读取供电电压(单位:V), PC0分压100K:10K, 实际电压=ADC电压×11
 * ADC 12位, 参考电压3.3V, Vsupply = adc/4095 × 3.3 × 11 ≈ adc × 0.00886 */
float MotorControl_GetSupplyVoltage(void)
{
    uint16_t adc_val = MotorControl_ReadADCAverage(1);
    return (float)adc_val * 3.3f * 11.0f / 4095.0f;
}

/* 读取供电电压ADC原始值(12位, 0~4095, PC0通道)
 * 供数据采集使用, 避免重复浮点运算 */
uint16_t MotorControl_GetSupplyVoltageADC(void)
{
    return MotorControl_ReadADCAverage(1);
}

/* 读取外部ADC原始值(12位, 0~4095)
 * channel: 0=PC2(外部输入1), 1=PC3(外部输入2) */
uint16_t MotorControl_GetExternalADC(uint8_t channel)
{
    if (channel == 0) {
        return MotorControl_ReadADCAverage(2);
    } else if (channel == 1) {
        return MotorControl_ReadADCAverage(3);
    }
    return 0;
}

/* 读取外部ADC带滤波值(新增)
 * 采用滑动平均: 维护size大小环形缓冲区, 每次用新值替换最老值, 返回窗口平均.
 * channel: 0=PC2, 1=PC3
 * size: 0或1=关闭滤波(直接返回原始值), 2~128=滑动平均窗口大小
 *       主循环5ms周期下, 100点≈500ms延迟, 128点≈640ms延迟(最大)
 * 注意:
 *   - size变化时会自动重置该通道的滤波器(清空缓冲区), 避免窗口不匹配错乱
 *   - 内部维护adc_filter_count, 启动阶段(count<size)用已填充样本数求平均
 *   - 累加和adc_filter_sum避免每次遍历求和, 复杂度O(1)
 *   - 调用频率: 主循环5ms一次(ADC模式), 性能完全足够
 *   - 注意: 此函数非static, 供modbus.c读取REG_EXT_ADC1/2时调用, 使上位机
 *     实时状态和采集折线图显示的也是滤波后值, 保持一致性 */
uint16_t MotorControl_GetExternalADCFiltered(MotorControl_t *mc, uint8_t channel)
{
    uint16_t raw = MotorControl_GetExternalADC(channel);
    uint8_t size = mc->adc_filter_size;

    /* 滤波关闭: 直接返回原始值 */
    if (size <= 1 || channel > 1) return raw;

    /* 窗口大小超限保护(2~ADC_FILTER_MAX_SIZE=128) */
    if (size > ADC_FILTER_MAX_SIZE) size = ADC_FILTER_MAX_SIZE;

    /* 关TIM1中断保护共享状态:
     * 此函数被3处调用: 主循环(MODBUS读REG_EXT_ADC1/2)、TIM1中断(ADC模式控制)、
     *   TIM1中断(采集speed_acq type=5/6). 主循环与TIM1中断并发访问同一通道的
     *   idx/count/sum会导致状态损坏(数据撕裂). 关中断保证原子性.
     * 关中断时间<10μs(仅几次内存操作), 对PID实时性无影响. */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

    /* 窗口大小变化时重置滤波器(检测size增大和减小两种情况)
     * 之前用 count>size 只能检测size减小, size增大时(count<size)会进入启动分支
     * 错误覆盖有效数据, 导致sum与buf不一致. 改用prev_size跟踪任意变化. */
    if (adc_filter_prev_size[channel] != size) {
        adc_filter_idx[channel] = 0;
        adc_filter_count[channel] = 0;
        adc_filter_sum[channel] = 0;
        adc_filter_prev_size[channel] = size;
    }

    /* 环形缓冲区: 新值覆盖最老值, 维护累加和 */
    uint8_t idx = adc_filter_idx[channel];
    if (adc_filter_count[channel] < size) {
        /* 启动阶段: 缓冲区未填满, 直接添加 */
        adc_filter_buf[channel][idx] = raw;
        adc_filter_sum[channel] += raw;
        adc_filter_count[channel]++;
    } else {
        /* 稳态: 替换最老值, 更新累加和 */
        adc_filter_sum[channel] -= adc_filter_buf[channel][idx];
        adc_filter_buf[channel][idx] = raw;
        adc_filter_sum[channel] += raw;
    }
    adc_filter_idx[channel] = (idx + 1) % size;

    /* 返回窗口平均 */
    uint16_t result = (uint16_t)(adc_filter_sum[channel] / adc_filter_count[channel]);

    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    return result;
}

/* 电流环PID: 返回PWM输出(-1000~+1000)
 * 由MotorControl_TimerTick100us每100μs调用一次
 * 电流采集和过流检测始终运行(无论current_loop_en是否使能):
 *   - current_loop_en=0: 读ADC + 过流检测 + max_current软件斩波 + current_target作为PWM
 *   - current_loop_en=1: 读ADC + 过流检测 + PID + 输出PWM */
static int16_t MotorControl_CurrentLoop(MotorControl_t *mc)
{
    /* 电机未运行: 清积分, 输出0, 但仍读ADC更新current_actual
     * 加电流死区: 电机停止时PWM=0, 理论上无电流, 残余读数来自
     * INA240A失调电压+ADC噪声+H桥漏电流, 属正常现象.
     * 死区阈值3相对单位≈25mA, 低于此值视为0, 使显示干净.
     * 注意: 死区仅用于停止状态显示, 不影响运行时PID精度. */
    if (!motor_running) {
        mc->current_integral = 0.0f;
        mc->current_prev_error = 0.0f;
        float raw = MotorControl_ReadCurrent(mc);
        mc->current_actual = (fabsf(raw) < 3.0f) ? 0.0f : raw;
        return 0;
    }

    /* 过流保护已触发: 持续关闭输出, 等待用户复位, 但仍读ADC
     * 同样加死区, 过流状态下PWM=0, 残余为噪声 */
    if (mc->over_current_tripped) {
        mc->current_integral = 0.0f;
        float raw = MotorControl_ReadCurrent(mc);
        mc->current_actual = (fabsf(raw) < 3.0f) ? 0.0f : raw;
        return 0;
    }

    /* 堵转保护已触发: 持续关闭输出, 等待用户复位
     * 与过流保护同样处理: 清积分 + 读ADC(加死区) + 返回0
     * 注意: 速度环MotorControl_SpeedLoop已检查stall_tripped并清current_target,
     *       但电流环此前漏检stall_tripped, 会导致堵转后current_target虽为0,
     *       电流环PID仍因残余电流(actual≠0)产生非零error并累加积分,
     *       输出非零PWM形成零目标附近震荡(如图中±60波形) */
    if (mc->stall_tripped) {
        mc->current_integral = 0.0f;
        mc->current_prev_error = 0.0f;
        float raw = MotorControl_ReadCurrent(mc);
        mc->current_actual = (fabsf(raw) < 3.0f) ? 0.0f : raw;
        return 0;
    }

    /* ===== 电流采集(始终运行, 无论电流环是否使能) ===== */
    float actual = MotorControl_ReadCurrent(mc);
    mc->current_actual = actual;

    /* 过流检测: 实际电流绝对值超过阈值时立即触发保护 */
    if (mc->over_current_limit > 0.0f && fabsf(actual) > mc->over_current_limit) {
        mc->over_current_tripped = 1;
        mc->current_integral = 0.0f;
        return 0;
    }

    uint8_t pwm_passthrough = !mc->current_loop_en;

    /* 电流限幅: 电流环开启时限制电流目标; PWM直通模式做100us软件斩波限流.
     * 与 over_current_limit 区别: 限幅会自动恢复, 不触发保护;
     *   over_current_limit 是检测实际电流超限后关闭输出(需复位).
     *   - 电流环使能: current_target是电流目标, max_current限制电流目标
     *   - PWM直通: current_target是PWM, actual超过max_current且仍同方向驱动时,
     *     本100us周期PWM置0; 电流降回限制内后自动恢复PWM, 形成软件斩波削峰. */
    float target = mc->current_target;
    /* 不截顶current_target: 电流限幅改为在PID输出后按比例缩小PWM, 不改变PID目标
     * 原逻辑: target被max_current截顶(36)→error=24→P=360(40%)→PWM偏低→速度下降
     * 新逻辑: target保持原值(可达900)→error=888→P=13320→PWM截顶900(满量程!)
     *         当实际电流超过max_current时, 按比例缩小PWM输出, 维持电流在限值
     * 这正是用户期望: 电流未达限制前PID与不限流时一致, 电流超限后直接削减PWM */

    /* ===== 电流死区(新增): 防止静止时细微电流 =====
     * 仅在电流目标近0(电机应该静止)且实际电流绝对值<死区阈值时生效:
     *   - 直接返回PWM=0, 跳过PID运算
     *   - 清零电流环积分和上次误差, 防止残留积分产生非零PWM
     *   - 将current_actual钳制为0, 使上位机显示干净(无残余电流读数)
     * 电机运行时(target明显非0)不受死区影响, 保证正常驱动能力.
     * 死区阈值current_dead_zone=0时关闭此功能, 保持原行为.
     *
     * 注意: 判断条件用 fabsf(target) < 1.0f 而非 target == 0.0f:
     *   - 位置模式到达死区时, 位置环虽然调用 SetOutput(0) 把 current_target 置0,
     *     但电流环每100μs运行一次, 位置环5ms才运行一次, 期间 current_target
     *     可能因浮点累加误差残留极小非0值(如1e-6), 严格相等判断会失效.
     *   - 1.0相对单位≈8.25mA, 远小于任何实际驱动电流, 作为"应该静止"的阈值安全.
     *   - 级联模式位置死区内速度环也已清 current_target=0, 同样满足此条件. */
    if (mc->current_dead_zone > 0.0f && fabsf(target) < 1.0f &&
        fabsf(actual) < mc->current_dead_zone) {
        /* 渐变衰减积分而非硬清零, 避免到位瞬间积分突变导致速度骤变.
         * 保留prev_error更新, D项可正常响应. */
        mc->current_integral *= 0.9f;
        if (fabsf(mc->current_integral) < 0.01f) {
            mc->current_integral = 0.0f;
        }
        mc->current_actual = 0.0f;
        return 0;
    }

    /* PWM直通: current_target作为PWM输出, max_current用作软件斩波削峰.
     * 只切断继续增大同方向电流的PWM; 反向PWM允许输出, 用于主动减小电流。 */
    if (pwm_passthrough) {
        int16_t pwm = (int16_t)target;
        if (pwm > mc->max_output) pwm = mc->max_output;
        if (pwm < -mc->max_output) pwm = -mc->max_output;
        if (mc->max_current > 0.0f && fabsf(actual) > mc->max_current &&
            ((actual > 0.0f && pwm > 0) || (actual < 0.0f && pwm < 0))) {
            return 0;
        }
        return pwm;
    }

    /* ===== 电流环PID运算 ===== */
    const float dt = 0.0001f;  /* 100μs */

    /* 注意: 电流限幅(max_current)不再截顶target, 而是在PID输出后按比例缩小PWM.
     * 电流环PID内部的积分分离带/积分限幅/输出限幅均基于max_output(PWM满量程),
     * 不能用max_current限制PWM输出 —— PWM是驱动量, 需要到满量程才能驱动足够电流.
     * 电流未达限制前PID与不限流时完全一致, 电流超限后PWM *= max_current/|actual| */
    float error = target - actual;

    /* 渐变式积分分离 + 积分限幅: 修复"加Ki电流不缓慢上升"问题
     *
     * 问题现象: 电流环Ki>0时, 大误差(error>分离带)下电流不缓慢上升, 永远达不到目标.
     * 根因: 原逻辑是硬分离带(|error|<band才积分), error=316>band=225时积分完全不累加,
     *   current_integral恒为0, I项恒为0, 纯P控制无法消除稳态误差, 电流锁死在低值.
     *   用户调大cur_Kp到3会引发高频振荡(电机啸叫), 因为P项过大且无积分阻尼.
     *
     * 修复: 改为渐变式积分(与速度环MotorControl_SpeedLoop一致):
     *   gain = band / (band + |error|)
     *   - |error|<<band: gain≈1, 全积分(消除稳态误差)
     *   - |error|≈band:  gain≈0.5, 半积分(温和参与)
     *   - |error|>>band: gain≈0, 几乎不积分(防windup, 但仍缓慢累积)
     * 这样大误差时积分按比例衰减累加, 电流会缓慢上升至目标, 最终靠积分消除稳态误差.
     * 同时保留抗启动抖动能力: 启动瞬间大误差时int_gain小, 积分缓慢累积不会瞬间饱和. */
    /* BUG修复(2026-07-22): 原代码 integral_sep_band = max_output * 0.25 = 225(PWM单位)
     *   但 error 是电流单位(0~max_current, 如0~60), 225 >> 60,
     *   int_gain = 225/(225+55) = 0.80, 积分80%强度激活→永久windup→PWM振荡.
     * 修复: 分离带改用电流单位, 与error同量级.
     *   max_current=60时 sep_band=30, error=55时 int_gain=30/85=0.35 (原0.80)
     *   不限流时仍用max_output比例(此时电流目标可达max_output量级) */
    float integral_sep_band;
    if (mc->max_current > 0.0f) {
        integral_sep_band = mc->max_current * 0.5f;
    } else {
        integral_sep_band = (float)mc->max_output * 0.25f;
    }

    /* 误差过零清积分: 修复Ki较大时到达位置不停止的BUG(积分饱和windup)
     *
     * 问题现象: Ki=22时, 点击到达预定位置后电机不停止, 来回振荡.
     * 根因: Ki大时积分限幅 = max_output/Ki = 900/22 = 40.9, 运动中积分快速饱和到40.9,
     *   I = 22×40.9 = 900(满PWM). 到达目标时current_target反向(从+363变负值),
     *   error从+316变-316, 但积分仍为+40.9. 放电需 40.9/(316×0.0001×0.416)≈311ms,
     *   期间I=+900而P=-316, 输出=+584(方向错误!), 电机被反向驱动过冲→振荡→永不停止.
     *   这是经典积分饱和(windup)问题.
     *
     * 修复: 检测error符号变化(过零), 立即清零积分. 位置环已有此逻辑(见MotorControl_Update
     *   1939-1944行), 电流环原本缺失. 添加后误差反向时积分立即归零, 仅靠P项快速响应新方向,
     *   避免饱和积分导致方向错误输出.
     *
     * BUG修复(2026-07-21): 原代码用 current_prev_error(=actual) 与 error 比较符号,
     *   但 actual 和 error 含义不同. 当 actual 因噪声/换向在0附近波动变负时,
     *   prev_error<0 而 error=target-actual>0, 误判过零→积分清零→PWM突降→电机停顿.
     *   现象: 运行中突然停止, 然后慢慢继续转动(积分重新累积).
     *   修复: 新增 prev_error_for_zero 专门存上次的error, 用于过零检测.
     *   current_prev_error 继续存 actual, 供D项微分使用. 两者职责分离.
     *   注: 本工程为单电机系统, static变量安全; MotorControl_CurrentLoop仅由TIM1中断调用. */
    /* [已关闭] 电流环误差过零清零积分:
     * 原逻辑在电流 error 变号时把 current_integral 硬置0, 导致 I=cur_Ki*current_integral
     * 突然跌落, PWM骤变, 电机运行中突然停顿再缓慢恢复(积分重新累积).
     * 现禁用此硬清零. 抗 windup 由渐变式积分分离+积分限幅+输出饱和回退承担.
     * 保留 prev_error_for_zero 变量更新, 便于后续按需重新启用. */
    static float prev_error_for_zero = 0.0f;  /* 上次error, 专用于过零检测(已停用, 备用) */
    /* if (prev_error_for_zero != 0.0f &&
        ((prev_error_for_zero > 0.0f && error < 0.0f) ||
         (prev_error_for_zero < 0.0f && error > 0.0f))) {
        mc->current_integral = 0.0f;
    } */
    (void)prev_error_for_zero;
    prev_error_for_zero = error;

    float abs_err = fabsf(error);
    float int_gain = integral_sep_band / (integral_sep_band + abs_err);
    mc->current_integral += error * dt * int_gain;
    /* 积分限幅: |I| ≤ max_output / Ki, 使I项最大=max_output(满PWM)
     * 抗饱和靠渐变式积分分离 + 积分限幅 + 输出饱和回退, 不靠缩小积分限幅
     * (原"过零清积分"已禁用, 避免积分突变导致电机转速骤变) */
    if (mc->cur_Ki > 1e-6f) {
        float int_limit = (float)mc->max_output / mc->cur_Ki;
        if (mc->current_integral > int_limit) mc->current_integral = int_limit;
        if (mc->current_integral < -int_limit) mc->current_integral = -int_limit;
    }

    float P = mc->cur_Kp * error;
    float I = mc->cur_Ki * mc->current_integral;
    float D = 0.0f;
    if (mc->cur_Kd > 0.0f) {
        D = -mc->cur_Kd * (actual - mc->current_prev_error) / dt;
    }
    mc->current_prev_error = actual;

    /* 速度前馈(反电动势补偿): PWM += Kff * velocity
     *
     * 解决问题: 限流时电流目标被max_current截顶→P项不足→PWM<满量程→速度低于目标
     *   限流0.5A: target截顶在60, P=15*(60-5)=825(89%), 速度3716(目标5000)
     *   不限流:   target可达65+, P=15*(65-6)=880(98%), 速度4077
     *   差距10%来源于P项被电流目标截顶限制
     *
     * 前馈原理: 电机高速旋转时反电动势Ke*ω增大, 需要更多PWM维持相同电流.
     *   PWM = R*I + Ke*ω (电阻压降 + 反电动势)
     *   前馈FF=Kff*velocity直接补偿反电动势项, 不受电流目标截顶影响.
     *   限流时P项=825, 前馈补上75→PWM=900(满量程), 速度达到不限流水平.
     *
     * 方向保护: 仅在驱动方向(target与velocity同号)时加前馈.
     *   制动方向(target<0而velocity>0)不加前馈, 避免抵抗制动.
     *
     * 标定: Kff = (目标PWM - P项) / 稳态速度
     *   =(900-825)/3716 ≈ 0.020, 建议范围0.015~0.025
     *   寄存器×10000, Modbus值=150~250 */
    float FF = 0.0f;
    if (mc->cur_Kff > 0.0f) {
        float vel = (float)mc->velocity;
        if ((target > 0.0f && vel > 0.0f) || (target < 0.0f && vel < 0.0f)) {
            FF = mc->cur_Kff * vel;
        }
    }

    /* 记录电流环调试数据(供上位机读取) */
    mc->cur_loop_error = error;
    mc->cur_loop_p = P;
    mc->cur_loop_i = I;
    mc->cur_loop_d = D;

    int32_t output = (int32_t)(P + I + D + FF);

    /* 限幅 + 抗积分饱和
     * 渐变式积分下, 抗饱和回退也用同样的int_gain, 保证加减对称.
     * 输出饱和时回退本次按int_gain比例累加的积分, 防止windup.
     * 输出限幅=max_output(PWM满量程), 电流限幅已在入口target处削峰 */
    int16_t limit = mc->max_output;
    if (output > limit) {
        output = limit;
        mc->current_integral -= error * dt * int_gain;
    }
    if (output < -limit) {
        output = -limit;
        mc->current_integral -= error * dt * int_gain;
    }

    /* 电流限幅: 实际电流超过限制时, 按比例缩小PWM输出(不改变PID内部状态)
     *
     * 原理: PWM是驱动量, 电流是被控量. 电流超限时直接削减PWM,
     *       而非截顶PID目标. 这样电流未达限制前PID有满量程输出能力.
     * 公式: output *= max_current / |actual|  (当|actual| > max_current且同方向时)
     * 效果: actual=max_current时ratio=1.0(不削减), actual=2×max_current时ratio=0.5
     *       电流越大削减越多, 平滑过渡, 无硬斩波的高频纹波
     * 抗windup: PID的max_output抗饱和已防止积分windup(PID输出饱和时积分回退),
     *   此处仅削减最终PWM, 不影响PID内部状态(integral等) */
    if (mc->max_current > 0.0f) {
        float abs_actual = fabsf(actual);
        if (abs_actual > mc->max_current) {
            /* 仅削减与电流同方向的PWM(驱动方向), 反向PWM(制动方向)不削减 */
            if ((actual > 0.0f && output > 0) || (actual < 0.0f && output < 0)) {
                float ratio = mc->max_current / abs_actual;
                output = (int32_t)(output * ratio);
            }
        }
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
        /* 渐变衰减而非硬清零, 避免位置死区进出时速度环积分突变
         * 导致current_target骤变→电流目标突变→速度突变.
         * 位置死区内speed_loop_active=0, 每ms衰减10%, 约20ms归零. */
        mc->speed_loop_integral *= 0.9f;
        if (fabsf(mc->speed_loop_integral) < 0.01f) {
            mc->speed_loop_integral = 0.0f;
        }
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

    /* 双套PID: 电流环使能时使用电流模式参数(增益更小), 关闭时使用电压模式参数 */
    float eff_spd_Kp = mc->current_loop_en ? mc->spd_Kp_cur : mc->spd_Kp;
    float eff_spd_Ki = mc->current_loop_en ? mc->spd_Ki_cur : mc->spd_Ki;
    float eff_spd_Kd = mc->current_loop_en ? mc->spd_Kd_cur : mc->spd_Kd;

    /* eff_limit = output_limit(满量程): 电流限幅不再截顶速度环输出,
     * 改由电流环PID输出后按比例缩小PWM. 速度环输出current_target可达满量程,
     * 电流环在电流超限时直接削减PWM, 维持电流在限值 */
    int32_t eff_limit = output_limit;

    /* 积分分离 + 积分限幅: 防止速度环积分饱和导致电流目标虚高
     *
     * 问题现象: 电流环使能(3环级联)时, 电流目标远高于实际电流(实测目标936mA vs 实际68mA,
     *   跟踪误差>90%), 但电机速度稳定(2219, 目标2222). 降速度环Ki会损失速度精度.
     * 根因: 速度环1ms周期无条件累加积分. 稳态下编码器量化噪声导致的小速度误差(10脉冲/秒)
     *   长期累积, speed_loop_integral涨到4680, I项=Ki*4680=936相对值=936mA.
     *   电机轻载只需68mA(8相对值), 积分虚高导致电流环长期大误差空转, 启动时叠加电流环
     *   积分则引发抖动. 原有抗饱和仅在输出>eff_limit(242相对值=2A)时回退, 但输出未饱和
     *   (约113相对值), 积分无限累积.
     *
     * 修复(实测验证迭代):
     *   1) 积分分离: |speed_error|<分离带时才积分(小误差精调), |error|>=分离带时不积分
     *      (大误差仅靠P项快速响应). 分离带=目标速度2%(最小20脉冲/秒=速度死区).
     *      稳态量化噪声约10脉冲/秒<20, 会积分消除稳态误差; 启动大误差不积分避免饱和.
     *      注: 第一版逻辑写反(大误差才积分)导致修复无效, 已纠正.
     *      注: 分离带曾用50, 但正常过冲误差~40<50会触发积分修正→I项下降→PWM下降→
     *      速度崩溃→慢恢复(3秒), 表现为周期性停顿. 改为20(=死区)后过冲40>20不触发积分.
     *   2) 积分限幅: |I|=|Ki*integral|≤20相对值(≈165mA). 积分仅消除稳态误差(摩擦/负载),
     *      不承担大电流目标. 20相对值远小于eff_limit(242), 确保P项主导, 电流目标合理. */
    float spd_sep_band = fabsf((float)mc->target_velocity) * 0.02f;
    if (spd_sep_band < 20.0f) spd_sep_band = 20.0f;
    if (fabsf(speed_error) < spd_sep_band) {
        mc->speed_loop_integral += speed_error * dt;
    }
    /* 积分限幅: |I| ≤ 20相对值(≈165mA), 仅消除稳态误差, 不承担大电流目标 */
    if (eff_spd_Ki > 1e-6f) {
        const float spd_int_I_limit = 20.0f;  /* 积分项最大贡献, 相对值 */
        float int_limit = spd_int_I_limit / eff_spd_Ki;
        if (mc->speed_loop_integral > int_limit) mc->speed_loop_integral = int_limit;
        if (mc->speed_loop_integral < -int_limit) mc->speed_loop_integral = -int_limit;
    }

    float P = eff_spd_Kp * speed_error;
    float I = eff_spd_Ki * mc->speed_loop_integral;
    float D = -eff_spd_Kd * (mc->velocity - mc->speed_loop_prev_vel) / dt;
    mc->speed_loop_prev_vel = mc->velocity;

    /* 记录速度环调试数据(供上位机读取) */
    mc->spd_loop_error = speed_error;
    mc->spd_loop_p = P;
    mc->spd_loop_i = I;
    mc->spd_loop_d = D;

    int32_t output = (int32_t)(P + I + D);

    /* 限幅 + 抗积分饱和
     * output作为current_target, 使用满量程output_limit(不再受max_current截顶).
     * 电流限幅改由电流环PID输出后按比例缩小PWM实现, 不影响速度环PID内部状态 */

    if (output > eff_limit) {
        output = eff_limit;
        if (fabsf(speed_error) < spd_sep_band) {
            mc->speed_loop_integral -= speed_error * dt;
        }
    }
    if (output < -eff_limit) {
        output = -eff_limit;
        if (fabsf(speed_error) < spd_sep_band) {
            mc->speed_loop_integral -= speed_error * dt;
        }
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
    /* 始终更新current_target, 供MotorControl_CurrentLoop读取
     * (current_loop_en=0时电流环直接返回current_target作为PWM,
     *  current_loop_en=1时电流环用current_target做PID) */
    mc->current_target = (float)output;

    /* 输出为0时渐变衰减电流环积分, 而非突然归零.
     * 突然归零会导致 I=cur_Ki*current_integral 瞬间跌落, PWM骤变,
     * 电机运行中(尤其是到位瞬间)速度突变.
     *
     * 渐变衰减: 每次将积分乘以衰减系数(0.9), 约10个100μs周期(1ms)衰减到35%,
     * 约5ms衰减到12%, 约20ms衰减到1%, 既避免突变又能在短时间内自然归零防震荡.
     * 仅在电流环使能时衰减(电流环关闭时积分无意义), 且仅衰减不硬清零.
     * 电机停止/堵转/过流等保护场景仍硬清零(在CurrentLoop入口处处理). */
    if (output == 0 && mc->current_loop_en) {
        mc->current_integral *= 0.9f;
        /* 衰减到极小值时直接归零, 避免浮点残差长期累积 */
        if (fabsf(mc->current_integral) < 0.01f) {
            mc->current_integral = 0.0f;
        }
    }

    /* 开环/待机/电机未运行时立即写PWM; 其他运行模式由100us电流路径下发PWM,
     * 这样非开环的电流环关闭模式也能执行过流检测和max_current软件斩波。 */
    if (mc->mode == OPENLOOP_MODE || mc->mode == ADC_OPENLOOP_MODE ||
        mc->mode == STANDBY_MODE || !motor_running) {
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

    /* ========== 双向复位状态机（堵转/传感器居中）==========
     * HOMING_BIDIR_NEG_SEARCH: 负方向搜索, 到达堵转/负限位后记录位置, 转正方向
     * HOMING_BIDIR_POS_SEARCH: 正方向搜索, 到达堵转/正限位后记录位置, 计算中点转居中
     * HOMING_BIDIR_CENTER: 运动到中点+偏置位置, 到位后清零编码器设原点 */
    if (homing_state == HOMING_BIDIR_NEG_SEARCH) {
        homing_ticks++;

        /* 检测停止条件: 传感器模式检测负限位, 堵转模式检测速度近0 */
        uint8_t stop = 0;
        if (mc->home_mode == HOME_MODE_BIDIR_SENSOR) {
            if (ReadLimitSwitch(mc, 1)) stop = 1;  /* 负方向限位(limit_dir=1) */
        } else {
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
            if (homing_stall_ticks >= HOMING_STALL_TICKS) stop = 1;
        }

        if (stop) {
            /* 记录负方向停止位置, 切换到正方向搜索 */
            bidir_neg_position = current_position;
            homing_state = HOMING_BIDIR_POS_SEARCH;
            homing_ticks = 0;
            homing_stall_ticks = 0;
            homing_start_position = current_position;  /* 重置起始位置用于正方向max_distance检测 */
            mc->target_position = current_position;    /* 重置目标位置, 消除限位钳制期间的位置误差累积 */
            mc->target_pos_frac = 0.0f;
            mc->target_speed = (float)mc->home_speed;  /* 正方向运动 */
            mc->integral = 0;
            mc->prev_error = 0;
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
            return 0;
        }

        /* 超过最大距离则失败 */
        int64_t distance = abs64(current_position - homing_start_position);
        if (distance >= mc->home_max_distance) {
            MotorControl_FinishHoming(mc, 1);
            return 0;
        }

        /* 持续负方向运动 */
        mc->target_speed = -(float)mc->home_speed;
    }

    if (homing_state == HOMING_BIDIR_POS_SEARCH) {
        homing_ticks++;

        /* 检测停止条件: 传感器模式检测正限位, 堵转模式检测速度近0 */
        uint8_t stop = 0;
        if (mc->home_mode == HOME_MODE_BIDIR_SENSOR) {
            if (ReadLimitSwitch(mc, 0)) stop = 1;  /* 正方向限位(limit_dir=0) */
        } else {
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
            if (homing_stall_ticks >= HOMING_STALL_TICKS) stop = 1;
        }

        if (stop) {
            /* 记录正方向停止位置, 计算中点+偏置, 切换到居中运动 */
            bidir_pos_position = current_position;
            int64_t center = (bidir_neg_position + bidir_pos_position) / 2;
            int64_t target = center + (int64_t)mc->home_back_distance;
            homing_state = HOMING_BIDIR_CENTER;
            mc->mode = POSITION_MODE;
            mc->target_position = target;
            mc->current_target = 0.0f;
            MotorControl_SetOutput(mc, 0);
            return 0;
        }

        /* 超过最大距离则失败 */
        int64_t distance = abs64(current_position - homing_start_position);
        if (distance >= mc->home_max_distance) {
            MotorControl_FinishHoming(mc, 1);
            return 0;
        }

        /* 持续正方向运动 */
        mc->target_speed = (float)mc->home_speed;
    }

    if (homing_state == HOMING_BIDIR_CENTER) {
        int64_t err = mc->target_position - current_position;
        if (abs64(err) <= 2) {
            MotorControl_FinishHoming(mc, 0);  /* 到位, 清零编码器设原点 */
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

        if (target_level_state != input) {
            target_level_state = input;
            if (mc->step_accel_en && !MotorControl_IsHoming()) {
                /* 步进加减速: 目标送入yudingweizhi, 算法生成平滑轨迹 */
                __disable_irq();
                bujindianji[0].yudingweizhi = target;
                __enable_irq();
                StepAccel_Restart();
            } else {
                MotorControl_SetTarget(mc, target);
                mc->target_pos_frac = 0.0f;
                MODBUS_SyncTargetPosition(mc->target_position);
            }
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
        /* 使用滤波后的ADC值(adc_filter_size=0或1时自动退化为原始值, 无副作用)
         * 滤波在死区处理之前应用, 避免噪声在死区边缘频繁触发 */
        uint16_t adc_raw = MotorControl_GetExternalADCFiltered(mc, 0);  /* PC2 */
        if (mc->pc2_func == ADC_FUNC_NONE) {
            active_func = mc->pc3_func;
            adc_raw = MotorControl_GetExternalADCFiltered(mc, 1);  /* PC3 */
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
            if (mc->step_accel_en && !MotorControl_IsHoming()) {
                /* 步进加减速: 目标送入yudingweizhi, 算法生成平滑轨迹 */
                int64_t ydwz;
                __disable_irq();
                ydwz = bujindianji[0].yudingweizhi;
                __enable_irq();
                if (ydwz != target) {
                    __disable_irq();
                    bujindianji[0].yudingweizhi = target;
                    __enable_irq();
                    StepAccel_Restart();
                }
            } else {
                if (mc->target_position != target) {
                    MotorControl_SetTarget(mc, target);
                    mc->target_pos_frac = 0.0f;
                    MODBUS_SyncTargetPosition(mc->target_position);
                }
            }
        }
    }

    /* 脉冲累加：将EXTI中断中检测到的脉冲边缘累加到目标位置 */
    if (pulse_accumulator != 0) {
        __disable_irq();
        int32_t pulses = pulse_accumulator;
        pulse_accumulator = 0;
        __enable_irq();
        if (mc->step_accel_en && MotorControl_IsPositionBasedMode(mc->mode) && !MotorControl_IsHoming()) {
            /* 步进加减速: 脉冲累加到yudingweizhi, 算法生成平滑轨迹 */
            __disable_irq();
            bujindianji[0].yudingweizhi += pulses;
            __enable_irq();
            StepAccel_Restart();
        } else {
            mc->target_position += pulses;
            MODBUS_SyncTargetPosition(mc->target_position);
        }
    }

    /* 基于位置的速度模式：目标位置按速度累加（浮点累加避免低速截断） */
    if (mc->mode == VELOCITY_POSITION_MODE || mc->mode == ADC_POSITION_SPEED_MODE) {
        float limited_speed = mc->target_speed;
        if (mc->max_run_speed > 0) limited_speed = clampf(limited_speed, -(float)mc->max_run_speed, (float)mc->max_run_speed);
        if (mc->step_accel_en && mc->mode == VELOCITY_POSITION_MODE && !MotorControl_IsHoming()) {
            /* 步进加减速: 速度累加到yudingweizhi, 算法生成平滑轨迹 */
            mc->target_pos_frac += limited_speed * dt;
            int64_t inc = (int64_t)mc->target_pos_frac;
            mc->target_pos_frac -= (float)inc;
            if (inc != 0) {
                __disable_irq();
                bujindianji[0].yudingweizhi += inc;
                __enable_irq();
                StepAccel_Restart();
            }
        } else {
            mc->target_pos_frac += limited_speed * dt;
            int64_t inc = (int64_t)mc->target_pos_frac;
            mc->target_position += inc;
            mc->target_pos_frac -= (float)inc;
            MODBUS_SyncTargetPosition(mc->target_position);
        }
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
        /* 步进加减速模式: 运动中(dangqiansudu≠0)跳过死区(目标持续移动, 清积分会导致卡顿),
         * 但到位后(dangqiansudu=0)必须应用死区, 否则残余位置误差→PWM饱和→极限环振荡.
         *
         * 100000步测试抖动根因: 步进完成后dangqiansudu=0但step_accel_en=1, 死区被跳过,
         * 60脉冲误差×有效增益(pos_Kp_cur×cur_Kp=35)=2100→PWM饱和900→bang-bang→振荡.
         * 修复: 检查dangqiansudu==0时视为"已完成", 允许死区介入停止PID输出. */
        uint8_t apply_deadzone = 0;
        if (mc->mode == POSITION_MODE || mc->mode == EXTERNAL_TARGET_MODE) {
            if (!mc->step_accel_en) {
                apply_deadzone = 1;
            } else {
                /* 步进加减速模式: 仅在算法停止时应用死区 */
                __disable_irq();
                int64_t step_spd = bujindianji[0].dangqiansudu;
                __enable_irq();
                if (step_spd == 0) apply_deadzone = 1;
            }
        }
        if (apply_deadzone && fabsf(error) <= mc->dead_zone) {
            /* 位置到位死区: 积分渐变衰减而非硬清零, 避免到位瞬间积分突变
             * 导致I项跌落→PWM骤变→速度突变.
             * mc->integral(位置环积分)和speed_limit_integral直接归零是安全的:
             *   位置环积分在死区内无意义(误差已为0), 速度限幅积分同理.
             * speed_loop_integral(1ms速度环)由SpeedLoop函数内根据
             *   speed_loop_active=0自行衰减, 此处不直接清零. */
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
        /* 双套PID: 电流环使能时使用电流模式位置Kp */
        float eff_pos_Kp = mc->current_loop_en ? mc->pos_Kp_cur : mc->pos_Kp;
        float target_vel = eff_pos_Kp * error;
        target_vel = clampf(target_vel, -(float)mc->max_run_speed, (float)mc->max_run_speed);
        mc->target_velocity = target_vel;
        mc->speed_loop_active = 1;  /* 激活1ms速度环 */
        mc->prev_error = error;
        mc->prev_velocity = mc->velocity;
        /* 调试信息: 记录位置环输出供上位机观察 */
        mc->pid_error = error;
        mc->pid_p = eff_pos_Kp * error;
        mc->pid_i = 0.0f;
        mc->pid_d = 0.0f;
        /* 电流目标由速度环(1ms)写入current_target, 此处不直接输出 */
        return 0;
    }

    /* 单环PID参数选择: 先按模式选速度/位置参数, 再按电流环状态选电压/电流模式参数 */
    uint8_t is_speed_mode = (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE ||
                             mc->mode == ADC_SPEED_MODE);
    float Kp = mc->current_loop_en ?
        (is_speed_mode ? mc->spd_Kp_cur : mc->pos_Kp_cur) :
        (is_speed_mode ? mc->spd_Kp : mc->pos_Kp);
    float Ki = mc->current_loop_en ?
        (is_speed_mode ? mc->spd_Ki_cur : mc->pos_Ki_cur) :
        (is_speed_mode ? mc->spd_Ki : mc->pos_Ki);
    float Kd = mc->current_loop_en ?
        (is_speed_mode ? mc->spd_Kd_cur : mc->pos_Kd_cur) :
        (is_speed_mode ? mc->spd_Kd : mc->pos_Kd);

    float P = Kp * error;
    mc->pid_error = error;

    /* [已关闭] 位置模式误差过零清零积分:
     * 原逻辑在位置误差变号瞬间把 mc->integral 硬置0, 导致 I=Ki*integral 突然跌落,
     * PID输出突变, 电机转速骤变(尤其Ki较大时). 现禁用此硬清零.
     * 抗 windup 改由下列机制共同承担, 足以替代过零清零:
     *   - 渐变式积分分离(int_gain = band/(band+|error|), 大误差时积分自动衰减)
     *   - 积分限幅(|I| <= max_output*系数/Ki)
     *   - 输出饱和时回退本次积分累积(见下方抗饱和块)
     * 注: VELOCITY_POSITION_MODE 本就跳过此逻辑; 步进加减速到位后由死区停机处理. */
    {
        uint8_t allow_zero_clear = (!mc->step_accel_en);
        if (mc->step_accel_en && (mc->mode == POSITION_MODE || mc->mode == EXTERNAL_TARGET_MODE)) {
            __disable_irq();
            int64_t step_spd_zc = bujindianji[0].dangqiansudu;
            __enable_irq();
            if (step_spd_zc == 0) allow_zero_clear = 1;
        }
        if ((mc->mode == POSITION_MODE || mc->mode == EXTERNAL_TARGET_MODE) && allow_zero_clear) {
            /* 过零清零已禁用: 避免积分突变导致电机转速骤变 */
            (void)allow_zero_clear;
            (void)error;
            /* if (mc->prev_error != 0.0f &&
                ((mc->prev_error > 0.0f && error < 0.0f) ||
                 (mc->prev_error < 0.0f && error > 0.0f))) {
                mc->integral = 0;
            } */
        }
    }

    /* 标准PID积分: integral累加 error*dt, I = Ki * integral
     *
     * 速度模式积分分离+限幅: 防止电流环使能时电流目标虚高(实测936mA vs 实际68mA).
     * 速度模式(mc->integral)与级联模式(speed_loop_integral)是两套独立积分变量,
     * 速度模式走此单环PID, 级联模式(位置+max_run_speed)走MotorControl_SpeedLoop.
     * 此前只改了SpeedLoop但速度模式走的是这里, 导致第一次修复无效.
     * 修复:
     *   1) 积分分离: 速度模式且|error|<分离带(目标速度2%,最小20脉冲/秒)时才积分,
     *      大误差仅靠P项. 稳态量化噪声(<20)会积分消除静差, 启动大误差不积分.
     *   2) 积分限幅: 速度模式|I|≤20相对值(≈165mA), 仅消除稳态摩擦/负载, 不承担大电流.
     *   3) 位置模式保持原逻辑(过零清零已防饱和, 积分用于消除位置静差). */
    if (is_speed_mode) {
        if (mc->current_loop_en) {
            /* 3环级联模式: 渐变式积分分离(替代硬分离带), 同时解决匀速停顿和变速卡顿.
             *
             * 问题历史:
             *   1) 匀速停顿: 分离带宽(2%)时, 正常速度波动(误差60~200)在带内积分,
             *      Ki=5~15导致I项波动大(0~20), 引发电流目标波动→速度振荡→周期性停顿.
             *   2) 变速卡顿: 分离带窄(0.5%)时, 大误差(1450)完全不积分, I项=0,
             *      仅靠P项(Kp=0.15)输出11~28, 加速缓慢, 变速卡顿0.6秒.
             *
             * 方案: 渐变式积分增益 - 误差小时全积分, 误差大时积分按比例衰减.
             *   gain = sep_band / (sep_band + |error|)   (0 < gain <= 1)
             *   - |error|<<sep_band: gain≈1, 全积分(消除稳态静差)
             *   - |error|≈sep_band: gain≈0.5, 半积分(温和参与)
             *   - |error|>>sep_band: gain≈0, 几乎不积分(防windup, 但非完全禁止)
             * 相比硬分离带, 大误差时积分缓慢参与, 帮助P项加速, 消除变速卡顿.
             *
             * sep_band=目标速度2%(最小20), 匀速时正常波动(60)的gain=20/80=0.25,
             * 积分贡献仅为全积分的1/4, 避免I项大幅波动导致停顿.
             * 积分限幅=max_current的0.8倍(或max_output的0.5倍), 确保I项有足够调节空间.
             * 注: 原限幅|I|≤15太小, Ki=5时integral限幅仅3, I项长期顶在15饱和失去调节
             *   能力, 速度突降时无法补偿, 导致匀速卡顿. 放宽到max_current的0.8倍后,
             *   I项可在0~190范围调节, 匀速时I项约12(轻载), 有充足余量应对扰动. */
            float spd_sep_band = fabsf((float)mc->target_speed) * 0.02f;
            if (spd_sep_band < 20.0f) spd_sep_band = 20.0f;
            float abs_err = fabsf(error);
            float int_gain = spd_sep_band / (spd_sep_band + abs_err);
            mc->integral += error * dt * int_gain;
            if (Ki > 1e-6f) {
                /* 积分限幅: 取max_current*0.8与max_output*0.5的较小值, 确保不超电流/输出限制 */
                float i_limit_val = (float)mc->max_output * 0.5f;
                if (mc->max_current > 0.0f && mc->max_current * 0.8f < i_limit_val) {
                    i_limit_val = mc->max_current * 0.8f;
                }
                float int_limit = i_limit_val / Ki;
                if (mc->integral > int_limit) mc->integral = int_limit;
                if (mc->integral < -int_limit) mc->integral = -int_limit;
            }
        } else {
            /* 2环模式(无电流环): output直接作为PWM, 积分需消除大静差(轻载电机需PWM~500).
             * 同样使用渐变式积分, sep_band=目标速度2%(最小50).
             * 积分限幅放宽到max_output*0.8/Ki, 允许积分贡献足够PWM消除31%静差. */
            float spd_sep_band = fabsf((float)mc->target_speed) * 0.02f;
            if (spd_sep_band < 50.0f) spd_sep_band = 50.0f;
            float abs_err = fabsf(error);
            float int_gain = spd_sep_band / (spd_sep_band + abs_err);
            mc->integral += error * dt * int_gain;
            if (Ki > 1e-6f) {
                float int_limit = (float)mc->max_output * 0.8f / Ki;
                if (mc->integral > int_limit) mc->integral = int_limit;
                if (mc->integral < -int_limit) mc->integral = -int_limit;
            }
        }
    } else {
        /* 位置模式积分: 步进加减速模式需限幅, 防止跟踪误差持续累加导致低频振荡
         *
         * 问题现象: 100000步加减速运行中, 匀速段速度有~3秒周期性抖动(波动7%),
         *   到位后5秒才稳定(pos_err从+129振荡到0), PWM在到位后持续负向(-5~-97).
         * 根因: 步进加减速模式下目标位置持续移动, 电机跟踪有微小滞后,
         *   位置误差持续为正→积分无限累加(I项达-308)→到位后积分需长时间放电→振荡.
         *   匀速段: 误差+57→积分增→PWM增→速度增→误差变负→积分减→PWM减→速度减→周期循环.
         *
         * 修复: 步进加减速模式下使用渐变式积分分离+限幅, 与速度模式一致:
         *   1) 分离带: 步进模式sep_band=30脉冲(位置死区30倍), |error|>band时积分按比例衰减
         *      正常跟踪误差5~20<30→全积分(消除稳态偏差), 大误差(启动/减速)>30→积分衰减
         *   2) 积分限幅: |I|≤max_output*0.1(=90相对值), 积分仅微调不主导
         *   非步进模式保持原逻辑(过零清零已防饱和, 积分用于消除位置静差) */
        if (mc->step_accel_en) {
            float pos_sep_band = 30.0f;  /* 位置积分分离带(脉冲) */
            float abs_err = fabsf(error);
            float int_gain = pos_sep_band / (pos_sep_band + abs_err);
            mc->integral += error * dt * int_gain;
            /* 积分限幅: |I| ≤ max_output*0.1, 防止积分主导输出 */
            if (Ki > 1e-6f) {
                float int_limit = (float)mc->max_output * 0.1f / Ki;
                if (mc->integral > int_limit) mc->integral = int_limit;
                if (mc->integral < -int_limit) mc->integral = -int_limit;
            }
        } else {
            mc->integral += error * dt;
        }
    }
    float I = Ki * mc->integral;
    mc->pid_p = P;
    mc->pid_i = I;

    /* 速度模式D项使用速度变化率, 减小噪声放大 */
    float D;
    if (mc->mode == SPEED_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE) {
        /* D项基于速度变化, 经过移动平均后噪声小 */
        D = -Kd * (mc->velocity - mc->prev_velocity) / dt;
    } else {
        /* 位置模式D项: 步进加减速模式用误差导数, 普通模式用速度阻尼
         *
         * 原代码: D = -Kd * velocity (所有位置模式统一)
         * 问题: 步进加减速模式目标持续移动, 匀速跟踪时velocity≈target_vel≠0,
         *   D=-Kd*4100=-234(制动力!), 积分必须windup到+772来克服, 导致~3秒周期抖动.
         *
         * 修复: 步进加减速模式改用误差导数 D=Kd*(target_vel-actual_vel)
         *   - 匀速跟踪: target_vel≈actual_vel → D≈0 (无多余制动力)
         *   - 启动加速: target_vel>actual_vel → D>0 (助力加速)
         *   - 减速停车: target_vel<actual_vel → D<0 (助力减速)
         * 普通位置模式(静止目标): target_vel=0 → D=-Kd*velocity (与原代码一致) */
        if (mc->step_accel_en) {
            float target_vel = (float)(mc->target_position - mc->prev_target_position) / dt;
            D = Kd * (target_vel - mc->velocity);
        } else {
            D = -Kd * mc->velocity;
        }
    }
    mc->pid_d = D;
    mc->prev_error = error;
    mc->prev_velocity = mc->velocity;

    /* 速度前馈(步进加减速模式): 提供维持速度所需的基础输出, 消除积分windup
     * FF = pos_Kff * target_vel
     * 匀速时FF提供主要驱动量, P/I/D仅修正微小偏差, 积分不再需要windup
     * 注意: 必须在prev_target_position更新之前计算, 否则target_vel=0 */
    float FF = 0.0f;
    if (mc->step_accel_en && mc->pos_Kff > 0.0f) {
        float target_vel_ff = (float)(mc->target_position - mc->prev_target_position) / dt;
        FF = mc->pos_Kff * target_vel_ff;
    }

    /* 更新prev_target_position(供下次D项和FF计算使用) */
    mc->prev_target_position = mc->target_position;

    int32_t output = (int32_t)(P + I + D + FF);

    /* 输出限幅 + 抗积分饱和:
     * 电流环使能时, output作为current_target, 不再受max_current限制,
     * 而是使用满量程max_output. 电流限幅改由电流环PID输出后按比例缩小PWM实现.
     * 这样位置/速度环可以输出满量程current_target, 电流环在电流超限时削减PWM */
    int32_t eff_limit = output_limit;

    if (output > eff_limit) {
        output = eff_limit;
        /* 饱和时回退本次积分累积(渐变式, 与上方累加逻辑一致) */
        if (is_speed_mode) {
            float spd_sep_band;
            if (mc->current_loop_en) {
                spd_sep_band = fabsf((float)mc->target_speed) * 0.02f;
                if (spd_sep_band < 20.0f) spd_sep_band = 20.0f;
            } else {
                spd_sep_band = fabsf((float)mc->target_speed) * 0.02f;
                if (spd_sep_band < 50.0f) spd_sep_band = 50.0f;
            }
            float int_gain = spd_sep_band / (spd_sep_band + fabsf(error));
            mc->integral -= error * dt * int_gain;
        } else if (mc->step_accel_en) {
            float pos_sep_band = 30.0f;
            float int_gain = pos_sep_band / (pos_sep_band + fabsf(error));
            mc->integral -= error * dt * int_gain;
        } else {
            mc->integral -= error * dt;
        }
        mc->pid_i = Ki * mc->integral;
    }
    if (output < -eff_limit) {
        output = -eff_limit;
        /* 饱和时回退本次积分累积(渐变式, 与上方累加逻辑一致) */
        if (is_speed_mode) {
            float spd_sep_band;
            if (mc->current_loop_en) {
                spd_sep_band = fabsf((float)mc->target_speed) * 0.02f;
                if (spd_sep_band < 20.0f) spd_sep_band = 20.0f;
            } else {
                spd_sep_band = fabsf((float)mc->target_speed) * 0.02f;
                if (spd_sep_band < 50.0f) spd_sep_band = 50.0f;
            }
            float int_gain = spd_sep_band / (spd_sep_band + fabsf(error));
            mc->integral -= error * dt * int_gain;
        } else if (mc->step_accel_en) {
            float pos_sep_band = 30.0f;
            float int_gain = pos_sep_band / (pos_sep_band + fabsf(error));
            mc->integral -= error * dt * int_gain;
        } else {
            mc->integral -= error * dt;
        }
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

    /* 启动前清零编码器(CNT/prev/total 原子清零, 实现已移至 encoder.c) */
    Encoder_Reset();

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* TIM3溢出中断已关闭: 编码器64位位置合成改在TIM1 100μs中断中
     * 通过差分累加实现(见Encoder_Synthesize), 不再依赖溢出中断修正wrap.
     * 这样彻底消除了"溢出已发生但中断未执行"窗口造成的累计误差. */
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

    /* 启动ADC DMA连续转换(4通道: 电流+电压+2个外部ADC)
     * ADC1已配置为连续转换模式+DMA循环模式, 启动后DMA按Rank顺序把4个通道的
     * 转换结果循环搬到adc_values[100]数组中, 各功能模块读取对应通道25个元素平均值
     * 注意: HAL_ADC_Start_DMA内部会调用HAL_DMA_Start_IT使能传输完成中断,
     * 但ADC连续转换每1us就会触发一次中断, 严重干扰TIM3编码器(0,0)和TIM1 PID(2,0)
     * 所以启动后立即关闭DMA1_Channel1的NVIC使能, 让DMA在后台静默传输 */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, ADC_DMA_VALUE_COUNT) != HAL_OK) {
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

/* 采集辅助函数: 根据采集类型返回一个int16_t采样值
 * 提取自TIM1中断中的采集逻辑, 供多通道轮询调用.
 * 参数: mc - 电机控制结构体, acq_type - 采集类型(0~23)
 * 返回: 钳位到int16_t范围的采样值 */
static int16_t MotorControl_GetAcqSample(MotorControl_t *mc, uint8_t acq_type)
{
    int16_t sample;
    if (acq_type == 1) {
        /* 采集PWM输出 (-1000~+1000) */
        sample = current_pwm_output;
    } else if (acq_type == 2) {
        /* 采集位置(相对起始位置的偏移, int16_t范围±32767脉冲) */
        int64_t diff = Encoder_GetCount() - mc->speed_acq_pos_start;
        if (diff > 32767) diff = 32767;
        if (diff < -32767) diff = -32767;
        sample = (int16_t)diff;
    } else if (acq_type == 3) {
        /* 采集电流(A×1000, 1A=1000, 满量程±8.25A=±8250) */
        float cur = MotorControl_ReadCurrent(mc);
        mc->current_actual = cur;
        float cur_A1000 = cur * 8.25f;
        if (cur_A1000 > 32767.0f) cur_A1000 = 32767.0f;
        if (cur_A1000 < -32767.0f) cur_A1000 = -32767.0f;
        sample = (int16_t)cur_A1000;
    } else if (acq_type == 4) {
        /* 采集PC0电压ADC原始值 (0~4095, 12位ADC) */
        sample = (int16_t)MotorControl_GetSupplyVoltageADC();
    } else if (acq_type == 5) {
        /* 采集PC2外部ADC值 (滤波后, 0~4095) */
        sample = (int16_t)MotorControl_GetExternalADCFiltered(mc, 0);
    } else if (acq_type == 6) {
        /* 采集PC3外部ADC值 (滤波后, 0~4095) */
        sample = (int16_t)MotorControl_GetExternalADCFiltered(mc, 1);
    } else if (acq_type >= 7 && acq_type <= 21) {
        /* 采集PID各环输出项(P/I/D实时输出) */
        float pid_val = 0.0f;
        uint8_t t = acq_type;
        if (t == 7 || t == 16) pid_val = mc->pid_p;
        else if (t == 8 || t == 17) pid_val = mc->pid_i;
        else if (t == 9 || t == 18) pid_val = mc->pid_d;
        else if (t == 10 || t == 19) pid_val = mc->spd_loop_p;
        else if (t == 11 || t == 20) pid_val = mc->spd_loop_i;
        else if (t == 12 || t == 21) pid_val = mc->spd_loop_d;
        else if (t == 13) pid_val = mc->cur_loop_p;
        else if (t == 14) pid_val = mc->cur_loop_i;
        else if (t == 15) pid_val = mc->cur_loop_d;
        if (pid_val > 32767.0f) pid_val = 32767.0f;
        if (pid_val < -32767.0f) pid_val = -32767.0f;
        sample = (int16_t)pid_val;
    } else if (acq_type == 22) {
        /* 采集位置误差 (目标位置 - 当前位置, 脉冲, int16_t范围±32767)
         * 位置模式下: pid_error即为位置误差, 直接读取
         * 其他模式下: 实时计算 target_position - Encoder_GetCount() */
        float err = (float)(mc->target_position - Encoder_GetCount());
        if (err > 32767.0f) err = 32767.0f;
        if (err < -32767.0f) err = -32767.0f;
        sample = (int16_t)err;
    } else if (acq_type == 23) {
        /* 采集速度误差 (目标速度 - 当前速度, 脉冲/秒, int16_t范围±32767)
         * 速度模式下: pid_error即为速度误差
         * 级联模式下: spd_loop_error为速度环误差(target_velocity - velocity)
         * 这里统一用 target_speed - velocity 计算, 适用于速度模式 */
        float err = mc->target_speed - mc->velocity;
        if (err > 32767.0f) err = 32767.0f;
        if (err < -32767.0f) err = -32767.0f;
        sample = (int16_t)err;
    } else {
        /* 默认: 采集转速 (脉冲/秒, int16_t范围±32767) */
        float spd = Encoder_GetSpeed();
        if (spd > 32767.0f) spd = 32767.0f;
        if (spd < -32767.0f) spd = -32767.0f;
        sample = (int16_t)spd;
    }
    return sample;
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

    /* 多通道数据采集：按分频值采样到缓冲区
     * 每个采样周期(divider个100us tick)依次采集所有通道,
     * 缓冲区数据交错存储: [ch1_t0, ch2_t0, ..., chN_t0, ch1_t1, ...]
     * 类似STM32 ADC扫描模式. 采样率(每通道)不变, 缓冲区填充快N倍. */
    if (motor_ctl && motor_ctl->speed_acq_active && !motor_ctl->speed_acq_done) {
        if (motor_ctl->speed_acq_div_cnt == 0) {
            /* 确定有效通道数(钳位1~4) */
            uint8_t nch = motor_ctl->speed_acq_channels;
            if (nch < 1) nch = 1;
            if (nch > 4) nch = 4;
            /* 依次采集每个通道 */
            uint8_t ch;
            for (ch = 0; ch < nch; ch++) {
                if (motor_ctl->speed_acq_count >= SPEED_ACQ_BUF_SIZE) break;
                if (motor_ctl->speed_acq_count >= motor_ctl->speed_acq_size) break;
                /* 选择当前通道的采集类型 */
                uint8_t acq_type;
                switch (ch) {
                    case 0:  acq_type = motor_ctl->speed_acq_type;  break;
                    case 1:  acq_type = motor_ctl->speed_acq_type2; break;
                    case 2:  acq_type = motor_ctl->speed_acq_type3; break;
                    case 3:  acq_type = motor_ctl->speed_acq_type4; break;
                    default: acq_type = 0; break;
                }
                int16_t sample = MotorControl_GetAcqSample(motor_ctl, acq_type);
                motor_ctl->speed_acq_buffer[motor_ctl->speed_acq_count] = sample;
                motor_ctl->speed_acq_count++;
            }
            /* 检查是否采满 */
            if (motor_ctl->speed_acq_count >= motor_ctl->speed_acq_size) {
                motor_ctl->speed_acq_done = 1;
                motor_ctl->speed_acq_active = 0;
            }
        }
        motor_ctl->speed_acq_div_cnt++;
        if (motor_ctl->speed_acq_div_cnt >= motor_ctl->speed_acq_divider) {
            motor_ctl->speed_acq_div_cnt = 0;
        }
    }

    /* ===== 电流路径: 每1次(100μs)执行 =====
     * 非开环、非待机模式经过这里:
     *   - current_loop_en=1: 读ADC + 过流检测 + 电流PID + 输出PWM
     *   - current_loop_en=0: 读ADC + 过流检测 + max_current软件斩波 + current_target作为PWM
     * 开环模式保持直接PWM输出, 不受max_current限制。 */
    if (motor_ctl && motor_ctl->mode != STANDBY_MODE &&
        motor_ctl->mode != OPENLOOP_MODE && motor_ctl->mode != ADC_OPENLOOP_MODE) {
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
            /* 步进加减速同步: 在位置PID运行前, 将dangqianweizhi复制到target_position
             * 原来在main.c主循环中异步执行(每1ms), 与5ms位置PID不同步,
             * 导致target_position时新时旧, 位置误差随机跳动.
             * 电流环使电机响应更快, 放大了这种跳动 → 转速抖动.
             * 移到此处与位置PID同步执行, 确保每次PID用的都是最新且一致的目标位置 */
            if (motor_ctl->step_accel_en &&
                MotorControl_IsPositionBasedMode(motor_ctl->mode) &&
                !MotorControl_IsHoming()) {
                int64_t step_pos;
                __disable_irq();
                step_pos = bujindianji[0].dangqianweizhi;
                __enable_irq();
                motor_ctl->target_position = step_pos;
            }
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

void MotorControl_SetMaxCurrent(MotorControl_t *mc, float max_current)
{
    /* 电流限幅由电流环PID实时读取, 写操作需关TIM1中断保证原子性.
     * 负值无意义, 强制为0(不限制). */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->max_current = (max_current < 0.0f) ? 0.0f : max_current;
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void MotorControl_SetCurrentDeadZone(MotorControl_t *mc, float dead_zone)
{
    /* 电流死区由电流环PID实时读取, 写操作需关TIM1中断保证原子性.
     * 负值无意义, 强制为0(关闭死区). */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    mc->current_dead_zone = (dead_zone < 0.0f) ? 0.0f : dead_zone;
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

/* 获取电流ADC平均值(12位, 0~4095, PC1通道)
 * 供上位机标定使用, 返回DMA缓冲区中该通道25个样本的平均值 */
uint16_t MotorControl_GetCurrentADC(void)
{
    return MotorControl_ReadADCAverage(0);
}

/* 自动零点标定: 电机停止时读取64次ADC值取平均作为零电流offset
 * 要求调用前电机已停止且无电流流过采样电阻
 * 返回新的offset值(0~4095), 同时更新mc->current_offset */
uint16_t MotorControl_AutoCalibrateCurrentOffset(MotorControl_t *mc)
{
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

    /* 采样64次取平均, 抑制噪声 */
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 64; i++) {
        sum += (uint32_t)MotorControl_ReadADCAverage(0);
        /* 简单延时等待ADC刷新(DMA连续转换, 等几个周期即可) */
        for (volatile uint16_t d = 0; d < 100; d++);
    }
    uint16_t new_offset = (uint16_t)(sum / 64);

    /* 更新offset, 使后续电流换算立即使用新零点 */
    mc->current_offset = new_offset;
    mc->current_actual = 0.0f;

    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    return new_offset;
}
