/**
  ******************************************************************************
  * @file    modbus.c
  * @brief   MODBUS RTU协议实现
  ******************************************************************************
  */

#include "modbus.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

/* 系统时间（100µs单位，TIM1中断中累加） */
volatile int64_t sys_time_100us = 0;

/* 启动标志位状态 */
static uint8_t start_flag_set = 0;
static uint16_t pending_control_word = 0;

/* 目标位置/速度缓存（MODBUS写入时暂存，ApplyTarget时读取） */
static int64_t pending_target_pos = 0;
static int32_t pending_target_speed = 0;

/* 脏标记：本次帧写入了需要保存的参数，帧处理后统一存Flash */
static uint8_t config_dirty = 0;
static uint32_t pending_baud_rate = 0;

/* 全局MODBUS上下文指针，供 MODBUS_TriggerStartFlag 等外部触发接口使用
 * 在 MODBUS_Init 中保存，主循环与外部触发同上下文，无需额外同步 */
static MB_Context_t *g_mb_ctx = NULL;

static const uint32_t supported_baud_rates[] = {9600, 19200, 38400, 57600, 115200};

uint8_t MODBUS_IsSupportedBaud(uint32_t baud_rate)
{
    for (uint8_t i = 0; i < sizeof(supported_baud_rates) / sizeof(supported_baud_rates[0]); i++) {
        if (supported_baud_rates[i] == baud_rate) {
            return 1;
        }
    }
    return 0;
}

uint16_t MODBUS_EncodeBaud(uint32_t baud_rate)
{
    return (uint16_t)(baud_rate / 100);
}

uint32_t MODBUS_DecodeBaud(uint16_t value)
{
    uint32_t baud_rate = (uint32_t)value * 100U;
    return MODBUS_IsSupportedBaud(baud_rate) ? baud_rate : 0;
}

uint16_t MODBUS_GetFrameTimeoutTicks(uint32_t baud_rate)
{
    uint32_t ticks = (350000U + baud_rate - 1U) / baud_rate;
    if (ticks < MB_FRAME_TIMEOUT_TICKS) {
        ticks = MB_FRAME_TIMEOUT_TICKS;
    }
    return (uint16_t)ticks;
}

void MODBUS_SyncTargetPosition(int64_t position)
{
    pending_target_pos = position;
}

HAL_StatusTypeDef MODBUS_ApplyBaudRate(MB_Context_t *ctx, uint32_t baud_rate)
{
    if (!MODBUS_IsSupportedBaud(baud_rate)) {
        return HAL_ERROR;
    }

    HAL_UART_AbortReceive(&huart1);
    HAL_UART_DeInit(&huart1);
    huart1.Init.BaudRate = baud_rate;
    HAL_StatusTypeDef status = HAL_UART_Init(&huart1);
    if (status == HAL_OK) {
        ctx->frame_timeout_ticks = MODBUS_GetFrameTimeoutTicks(baud_rate);
        ctx->timeout_ticks = 0;
        ctx->rx_index = 0;
        ctx->rx_len = 0;
        ctx->rx_complete = 0;
        HAL_UART_Receive_IT(&huart1, &ctx->rx_byte, 1);
    }
    return status;
}

/* CRC16查表法 */
static const uint16_t crc_table[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/**
  * @brief  计算CRC16
  */
static uint16_t MODBUS_CalcCRC(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

/**
  * @brief  初始化MODBUS上下文
  */
void MODBUS_Init(MB_Context_t *ctx, uint8_t slave_addr, MotorControl_t *motor)
{
    ctx->rx_index = 0;
    ctx->rx_len = 0;
    ctx->timeout_ticks = 0;
    ctx->frame_timeout_ticks = MODBUS_GetFrameTimeoutTicks(motor->modbus_baud);
    ctx->rx_complete = 0;
    ctx->slave_addr = slave_addr;
    ctx->motor = motor;
    ctx->rx_count = 0;
    ctx->error_count = 0;
    start_flag_set = 0;
    pending_control_word = motor->mode & CTRL_MODE_MASK;
    pending_target_pos = 0;
    pending_target_speed = 0;
    config_dirty = 0;
    pending_baud_rate = 0;
    g_mb_ctx = ctx;  /* 保存上下文指针供外部触发接口使用 */
}

/**
  * @brief  UART中断中调用：存入一个字节并重置超时计数
  */
void MODBUS_OnByteReceived(MB_Context_t *ctx, uint8_t byte)
{
    if (ctx->rx_index < MB_RX_BUF_SIZE) {
        ctx->rx_buf[ctx->rx_index] = byte;
        ctx->rx_index++;
    }
    /* 每收到一个字节，重置帧间超时计数 */
    ctx->timeout_ticks = 0;
}

/**
  * @brief  TIM1中断中调用（100us周期）：超时检测帧结束
  *
  * 原理：MODBUS RTU通过3.5字符时间的空闲间隔判定帧边界。
  *       每收到一字节重置timeout_ticks，当计数值达到阈值时
  *       说明总线已空闲超过3.5字符时间，当前帧接收完成。
  */
void MODBUS_OnTimeoutTick(MB_Context_t *ctx)
{
    /* 没有正在接收的数据，不计时 */
    if (ctx->rx_index == 0) {
        ctx->timeout_ticks = 0;
        return;
    }

    ctx->timeout_ticks++;

    /* 达到帧间超时，判定一帧接收完成 */
    if (ctx->timeout_ticks >= ctx->frame_timeout_ticks) {
        if (ctx->rx_complete) {
            /* 上一帧还未被主循环处理，丢弃当前帧 */
            ctx->error_count++;
            ctx->rx_index = 0;
            ctx->timeout_ticks = 0;
            return;
        }

        /* 复制到处理缓冲区供主循环处理 */
        uint16_t len = ctx->rx_index;
        if (len > MB_RX_BUF_SIZE) {
            len = MB_RX_BUF_SIZE;
        }
        memcpy(ctx->rx_process_buf, ctx->rx_buf, len);
        ctx->rx_len = len;
        ctx->rx_complete = 1;
        ctx->rx_count++;
        ctx->rx_index = 0;
        ctx->timeout_ticks = 0;
    }
}

/**
  * @brief  发送响应
  */
void MODBUS_SendResponse(MB_Context_t *ctx, uint8_t *data, uint8_t len)
{
    /* 添加CRC */
    uint16_t crc = MODBUS_CalcCRC(data, len);
    data[len++] = crc & 0xFF;        /* CRC低字节 */
    data[len++] = (crc >> 8) & 0xFF; /* CRC高字节 */

    /* RS485方向控制: DE=1发送, 发送完成后立即回到接收 */
    HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_TC);
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, data, len, 100);
    uint32_t start_tick = HAL_GetTick();
    while (status == HAL_OK && __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
        if (HAL_GetTick() - start_tick > 10U) {
            break;
        }
    }
    HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  发送异常响应
  */
static void MODBUS_SendException(MB_Context_t *ctx, uint8_t func, uint8_t exception)
{
    uint8_t response[3];
    response[0] = ctx->slave_addr;
    response[1] = func | 0x80;  /* 异常标志 */
    response[2] = exception;
    MODBUS_SendResponse(ctx, response, 3);
}

/**
  * @brief  读取寄存器值
  */
static uint16_t MODBUS_ReadRegister(MB_Context_t *ctx, uint16_t addr)
{
    MotorControl_t *mc = ctx->motor;
    
    /* 状态寄存器（只读） */
    if (addr >= 0x0100 && addr <= 0x0115) {
        switch (addr) {
            case REG_CURRENT_POS_H3:
                return (uint16_t)(Encoder_GetCount() >> 48);
            case REG_CURRENT_POS_H2:
                return (uint16_t)(Encoder_GetCount() >> 32);
            case REG_CURRENT_POS_L2:
                return (uint16_t)(Encoder_GetCount() >> 16);
            case REG_CURRENT_POS_L1:
                return (uint16_t)(Encoder_GetCount() & 0xFFFF);
            case REG_CURRENT_SPEED_H:
                return (uint16_t)((int32_t)Encoder_GetSpeed() >> 16);
            case REG_CURRENT_SPEED_L:
                return (uint16_t)((int32_t)Encoder_GetSpeed() & 0xFFFF);
            case REG_CURRENT_MODE:
                return MotorControl_GetMode(mc);
            case REG_STATUS: {
                /* 状态字位定义见 modbus.h STATUS_xxx 宏
                 * PB4/PB5电平直接读GPIOB IDR, 与引脚功能配置无关,
                 * 上位机可实时显示引脚当前电平用于调试 */
                uint8_t pb4 = (GPIOB->IDR & GPIO_PIN_4) ? 1 : 0;
                uint8_t pb5 = (GPIOB->IDR & GPIO_PIN_5) ? 1 : 0;
                uint16_t status = (start_flag_set ? STATUS_START_FLAG : 0) |
                                  (MotorControl_IsHoming() ? STATUS_HOMING : 0) |
                                  (MotorControl_HomingFailed() ? STATUS_HOMING_FAILED : 0) |
                                  (MotorControl_IsStallTripped() ? STATUS_STALL_TRIPPED : 0) |
                                  (MotorControl_IsOverCurrentTripped() ? STATUS_OVER_CUR_TRIPPED : 0) |
                                  (pb4 ? STATUS_PB4_LEVEL : 0) |
                                  (pb5 ? STATUS_PB5_LEVEL : 0) |
                                  (mc->mode & CTRL_MODE_MASK);
                return status;
            }
            case REG_CURRENT_PWM:
                return (uint16_t)MotorControl_GetPWM();
            case REG_PID_ERROR_H:
                return (uint16_t)((int32_t)(mc->pid_error * 100) >> 16);
            case REG_PID_ERROR_L:
                return (uint16_t)((int32_t)(mc->pid_error * 100) & 0xFFFF);
            case REG_PID_P_H:
                return (uint16_t)((int32_t)(mc->pid_p * 100) >> 16);
            case REG_PID_P_L:
                return (uint16_t)((int32_t)(mc->pid_p * 100) & 0xFFFF);
            case REG_PID_I_H:
                return (uint16_t)((int32_t)(mc->pid_i * 100) >> 16);
            case REG_PID_I_L:
                return (uint16_t)((int32_t)(mc->pid_i * 100) & 0xFFFF);
            case REG_PID_D_H:
                return (uint16_t)((int32_t)(mc->pid_d * 100) >> 16);
            case REG_PID_D_L:
                return (uint16_t)((int32_t)(mc->pid_d * 100) & 0xFFFF);
            case REG_CURRENT_ACTUAL:
                /* 实际电流(相对值×10, 有符号) */
                return (uint16_t)(int16_t)(mc->current_actual * 10);
            case REG_CURRENT_TARGET_RO:
                /* 电流目标(相对值×10, 有符号) */
                return (uint16_t)(int16_t)(mc->current_target * 10);
            case REG_SUPPLY_VOLTAGE:
                /* 供电电压(单位0.01V), PC0分压100K:10K, 实际电压=ADC电压×11
                 * 例: 3300 = 33.00V, 2400 = 24.00V, 上限约363.0V (12位ADC满量程) */
                return (uint16_t)(MotorControl_GetSupplyVoltage() * 100.0f + 0.5f);
            case REG_EXT_ADC1:
                /* 外部ADC1原始值(PC2, 12位 0~4095) */
                return MotorControl_GetExternalADC(0);
            case REG_EXT_ADC2:
                /* 外部ADC2原始值(PC3, 12位 0~4095) */
                return MotorControl_GetExternalADC(1);
        }
    }

    /* 转速采集数据寄存器（只读） 0x0200~0x03FF */
    if (addr >= REG_SPEED_DATA_BASE && addr <= REG_SPEED_DATA_END) {
        uint16_t idx = addr - REG_SPEED_DATA_BASE;
        if (idx < mc->speed_acq_count) {
            return (uint16_t)mc->speed_acq_buffer[idx];
        }
        return 0;  /* 未采集到的位置返回0 */
    }

    /* 转速采集控制/状态寄存器 0x003A~0x003F */
    if (addr >= REG_SPEED_ACQ_START && addr <= REG_SPEED_ACQ_SIZE) {
        switch (addr) {
            case REG_SPEED_ACQ_START:
            case REG_SPEED_ACQ_STATUS:
                return mc->speed_acq_done ? 2 : (mc->speed_acq_active ? 1 : 0);
            case REG_SPEED_ACQ_DIV:
                return mc->speed_acq_divider;
            case REG_SPEED_ACQ_COUNT:
                return mc->speed_acq_count;
            case REG_SPEED_ACQ_TYPE:
                return mc->speed_acq_type;
            case REG_SPEED_ACQ_SIZE:
                return mc->speed_acq_size;
        }
    }

    /* 堵转保护寄存器 0x0040~0x0044 */
    if (addr >= REG_STALL_PROT_EN && addr <= REG_STALL_RESET) {
        switch (addr) {
            case REG_STALL_PROT_EN:
                return mc->stall_protect_en ? 1 : 0;
            case REG_STALL_ERR_LIMIT:
                return (uint16_t)mc->stall_err_limit;
            case REG_STALL_TIME:
                return mc->stall_time_ticks;
            case REG_STALL_STATUS:
                return mc->stall_tripped ? 1 : 0;
            case REG_STALL_RESET:
                return 0;  /* 写触发型寄存器, 读取始终返回0 */
        }
    }

    /* 电流环寄存器 0x0045~0x004D (新增) */
    if (addr >= REG_CUR_LOOP_EN && addr <= REG_OVER_CUR_RESET) {
        switch (addr) {
            case REG_CUR_LOOP_EN:
                return mc->current_loop_en ? 1 : 0;
            case REG_CUR_KP:
                return (uint16_t)(int16_t)(mc->cur_Kp * 100);
            case REG_CUR_KI:
                return (uint16_t)(int16_t)(mc->cur_Ki * 100);
            case REG_CUR_KD:
                return (uint16_t)(int16_t)(mc->cur_Kd * 1000);  /* D项×1000, 3位小数 */
            case REG_CUR_OFFSET:
                return mc->current_offset;
            case REG_CUR_SCALE:
                return (uint16_t)(mc->current_scale * 10000);  /* 4位小数 */
            case REG_OVER_CUR_LIMIT:
                return (uint16_t)(int16_t)(mc->over_current_limit * 10);  /* ×10, 1位小数 */
            case REG_OVER_CUR_STATUS:
                return mc->over_current_tripped ? 1 : 0;
            case REG_OVER_CUR_RESET:
                return 0;  /* 写触发型寄存器, 读取始终返回0 */
        }
    }

    /* PC2/PC3 ADC功能寄存器 0x004E~0x0061 (新增双死区) */
    if (addr >= REG_PC2_FUNC && addr <= REG_ADC_DEAD_ZONE2_WIDTH) {
        switch (addr) {
            case REG_PC2_FUNC:
                return mc->pc2_func;
            case REG_PC3_FUNC:
                return mc->pc3_func;
            case REG_ADC_MAX_SPEED_H:
                return (uint16_t)((int32_t)mc->adc_max_speed >> 16);
            case REG_ADC_MAX_SPEED_L:
                return (uint16_t)((int32_t)mc->adc_max_speed & 0xFFFF);
            case REG_ADC_MAX_PWM:
                return (uint16_t)(int16_t)mc->adc_max_pwm;
            case REG_ADC_MAX_POS_H3:
                return (uint16_t)((uint64_t)mc->adc_max_position >> 48);
            case REG_ADC_MAX_POS_H2:
                return (uint16_t)((uint64_t)mc->adc_max_position >> 32);
            case REG_ADC_MAX_POS_L2:
                return (uint16_t)((uint64_t)mc->adc_max_position >> 16);
            case REG_ADC_MAX_POS_L1:
                return (uint16_t)((uint64_t)mc->adc_max_position & 0xFFFF);
            /* ADC最小值寄存器(新增) */
            case REG_ADC_MIN_SPEED_H:
                return (uint16_t)((int32_t)mc->adc_min_speed >> 16);
            case REG_ADC_MIN_SPEED_L:
                return (uint16_t)((int32_t)mc->adc_min_speed & 0xFFFF);
            case REG_ADC_MIN_PWM:
                return (uint16_t)(int16_t)mc->adc_min_pwm;
            case REG_ADC_MIN_POS_H3:
                return (uint16_t)((uint64_t)mc->adc_min_position >> 48);
            case REG_ADC_MIN_POS_H2:
                return (uint16_t)((uint64_t)mc->adc_min_position >> 32);
            case REG_ADC_MIN_POS_L2:
                return (uint16_t)((uint64_t)mc->adc_min_position >> 16);
            case REG_ADC_MIN_POS_L1:
                return (uint16_t)((uint64_t)mc->adc_min_position & 0xFFFF);
            /* ADC死区寄存器(新增, 双死区) */
            case REG_ADC_DEAD_ZONE1_POS:
                return mc->adc_dead_zone1_pos;
            case REG_ADC_DEAD_ZONE1_WIDTH:
                return mc->adc_dead_zone1_width;
            case REG_ADC_DEAD_ZONE2_POS:
                return mc->adc_dead_zone2_pos;
            case REG_ADC_DEAD_ZONE2_WIDTH:
                return mc->adc_dead_zone2_width;
        }
    }
    
    /* 控制寄存器（可读写） */
    if (addr >= 0x0000 && addr <= 0x0038) {
        switch (addr) {
            case REG_CONTROL_WORD:
                return pending_control_word & CTRL_MODE_MASK;
            case REG_MODE_SET:
                return mc->start_mode ? 1 : 0;
            case REG_TARGET_POS_H3:
                return (uint16_t)(pending_target_pos >> 48);
            case REG_TARGET_POS_H2:
                return (uint16_t)(pending_target_pos >> 32);
            case REG_TARGET_POS_L2:
                return (uint16_t)(pending_target_pos >> 16);
            case REG_TARGET_POS_L1:
                return (uint16_t)(pending_target_pos & 0xFFFF);
            case REG_TARGET_SPEED_H:
                return (uint16_t)((int32_t)pending_target_speed >> 16);
            case REG_TARGET_SPEED_L:
                return (uint16_t)((int32_t)pending_target_speed & 0xFFFF);
            case REG_POS_KP:
                return (uint16_t)(int16_t)(mc->pos_Kp * 100);
            case REG_POS_KI:
                return (uint16_t)(int16_t)(mc->pos_Ki * 100);
            case REG_POS_KD:
                return (uint16_t)(int16_t)(mc->pos_Kd * 1000);  /* D项×1000, 3位小数 */
            case REG_SPD_KP:
                return (uint16_t)(int16_t)(mc->spd_Kp * 100);
            case REG_SPD_KI:
                return (uint16_t)(int16_t)(mc->spd_Ki * 100);
            case REG_SPD_KD:
                return (uint16_t)(int16_t)(mc->spd_Kd * 1000);  /* D项×1000, 3位小数 */
            case REG_DEAD_ZONE:
                return mc->dead_zone;
            case REG_MAX_OUTPUT:
                return mc->max_output;
            case REG_START_MODE:
                return start_flag_set ? 1 : 0;
            case REG_DIRECTION:
                return (uint16_t)mc->direction;
            case REG_ENCODER_DIRECTION:
                return (uint16_t)mc->encoder_dir;
            case REG_START_FLAG:
                return start_flag_set ? 1 : 0;
            case REG_SLAVE_ADDR:
                return ctx->slave_addr;
            case REG_BAUD_RATE:
                return MODBUS_EncodeBaud(mc->modbus_baud);
            case REG_SET_ORIGIN:
                return 0;
            case REG_HOME_MODE:
                return mc->home_mode;
            case REG_HOME_DIRECTION:
                return (uint16_t)mc->home_dir;
            case REG_HOME_CURRENT:
                return (uint16_t)mc->home_current;
            case REG_HOME_SPEED_H:
                return (uint16_t)((uint32_t)mc->home_speed >> 16);
            case REG_HOME_SPEED_L:
                return (uint16_t)((uint32_t)mc->home_speed & 0xFFFF);
            case REG_HOME_MAX_DISTANCE_H:
                return (uint16_t)((uint32_t)mc->home_max_distance >> 16);
            case REG_HOME_MAX_DISTANCE_L:
                return (uint16_t)((uint32_t)mc->home_max_distance & 0xFFFF);
            case REG_HOME_BACK_DISTANCE_H:
                return (uint16_t)((uint32_t)mc->home_back_distance >> 16);
            case REG_HOME_BACK_DISTANCE_L:
                return (uint16_t)((uint32_t)mc->home_back_distance & 0xFFFF);
            case REG_HOME_TRIGGER:
                return 0;
            case REG_HOME_AUTO_START:
                return mc->home_auto_start;
            case REG_INPUT1_FUNC:
                return mc->pin4_func;
            case REG_INPUT1_POLARITY:
                return mc->pin4_polarity;
            case REG_INPUT1_LIMIT_DIR:
                return mc->pin4_limit_dir;
            case REG_INPUT2_FUNC:
                return mc->pin5_func;
            case REG_INPUT2_POLARITY:
                return mc->pin5_polarity;
            case REG_INPUT2_LIMIT_DIR:
                return mc->pin5_limit_dir;
            case REG_MAX_RUN_SPEED_H:
                return (uint16_t)((uint32_t)mc->max_run_speed >> 16);
            case REG_MAX_RUN_SPEED_L:
                return (uint16_t)((uint32_t)mc->max_run_speed & 0xFFFF);
            case REG_INPUT1_TARGET_POS_H3:
                return (uint16_t)(mc->pin4_target_pos >> 48);
            case REG_INPUT1_TARGET_POS_H2:
                return (uint16_t)(mc->pin4_target_pos >> 32);
            case REG_INPUT1_TARGET_POS_L2:
                return (uint16_t)(mc->pin4_target_pos >> 16);
            case REG_INPUT1_TARGET_POS_L1:
                return (uint16_t)(mc->pin4_target_pos & 0xFFFF);
            case REG_INPUT2_TARGET_POS_H3:
                return (uint16_t)(mc->pin5_target_pos >> 48);
            case REG_INPUT2_TARGET_POS_H2:
                return (uint16_t)(mc->pin5_target_pos >> 32);
            case REG_INPUT2_TARGET_POS_L2:
                return (uint16_t)(mc->pin5_target_pos >> 16);
            case REG_INPUT2_TARGET_POS_L1:
                return (uint16_t)(mc->pin5_target_pos & 0xFFFF);
            case REG_HOME_PRECISION_SPEED_H:
                return (uint16_t)((uint32_t)mc->home_precision_speed >> 16);
            case REG_HOME_PRECISION_SPEED_L:
                return (uint16_t)((uint32_t)mc->home_precision_speed & 0xFFFF);
            case REG_HOME_PRECISION_CYCLES:
                return mc->home_precision_cycles;
            case REG_INPUT1_TARGET_SPEED_H:
                return (uint16_t)((int32_t)mc->pin4_target_speed >> 16);
            case REG_INPUT1_TARGET_SPEED_L:
                return (uint16_t)((int32_t)mc->pin4_target_speed & 0xFFFF);
            case REG_INPUT2_TARGET_SPEED_H:
                return (uint16_t)((int32_t)mc->pin5_target_speed >> 16);
            case REG_INPUT2_TARGET_SPEED_L:
                return (uint16_t)((int32_t)mc->pin5_target_speed & 0xFFFF);
        }
    }
    
    return 0;
}

/**
  * @brief  应用目标值（位置/速度）
  *         直接从pending缓存读取，而非mc->target_position（那是当前已生效值）
  */
static void MODBUS_ApplyTarget(MB_Context_t *ctx)
{
    MotorControl_t *mc = ctx->motor;

    if (mc->mode == POSITION_MODE || mc->mode == EXTERNAL_TARGET_MODE) {
        MotorControl_SetTarget(mc, pending_target_pos);
    } else if (mc->mode == SPEED_MODE || mc->mode == VELOCITY_POSITION_MODE || mc->mode == OPENLOOP_MODE || mc->mode == EXTERNAL_TARGET_SPEED_MODE) {
        MotorControl_SetSpeed(mc, (float)pending_target_speed);
    }
}

static void MODBUS_ApplyMode(MB_Context_t *ctx, uint8_t mode, uint8_t sync_current)
{
    MotorControl_t *mc = ctx->motor;

    MotorControl_SetMode(mc, mode);
    if (sync_current) {
        if (mode == POSITION_MODE || mode == VELOCITY_POSITION_MODE || mode == EXTERNAL_TARGET_MODE) {
            pending_target_pos = Encoder_GetCount();
            pending_target_speed = 0;
        } else if (mode == SPEED_MODE || mode == OPENLOOP_MODE || mode == EXTERNAL_TARGET_SPEED_MODE) {
            pending_target_speed = 0;
        }
    }
    MODBUS_ApplyTarget(ctx);
}

/* 触发启动标志位 (等同于上位机写 REG_START_MODE=1)
 * 供 PB4/PB5 配置为 PIN_FUNC_START_FLAG 时由硬件有效边沿触发调用,
 * 实现"上位机批量下发目标 + 硬件信号同步启动"的多电机同步启动场景:
 *   1. 上位机向各电机(标志位启动模式)下发模式/目标位置/目标速度(仅缓存不执行)
 *   2. 同一硬件信号接入各电机 PB4/PB5, 信号跳变时各电机同时应用缓存目标
 * 仅当 start_mode=1 时有效; start_mode=0(直接启动)时调用本函数无动作. */
void MODBUS_TriggerStartFlag(void)
{
    if (!g_mb_ctx) return;
    MotorControl_t *mc = g_mb_ctx->motor;
    if (mc->start_mode) {
        MotorMode_t old_mode = mc->mode;
        start_flag_set = 1;
        MODBUS_ApplyMode(g_mb_ctx, pending_control_word & CTRL_MODE_MASK, 0);
        if (mc->mode != old_mode) {
            config_dirty = 1;
        }
    }
}

/**
  * @brief  写入寄存器值
  * @param  ctx: MODBUS上下文
  * @param  addr: 寄存器地址
  * @param  value: 寄存器值
  * @param  is_multiple: 是否为多寄存器写入（多寄存器写入时不立即启动）
  */
static void MODBUS_WriteRegister(MB_Context_t *ctx, uint16_t addr, uint16_t value, uint8_t is_multiple)
{
    MotorControl_t *mc = ctx->motor;
    uint8_t need_apply = 0;  /* 标记是否需要立即应用目标值 */
    
    /* 只读寄存器 */
    if (addr >= 0x0100 && addr <= 0x0115) {
        return;
    }
    /* 转速数据寄存器只读 0x0200~0x03FF */
    if (addr >= REG_SPEED_DATA_BASE && addr <= REG_SPEED_DATA_END) {
        return;
    }
    
    /* 控制寄存器 */
    if (addr >= 0x0000 && addr <= 0x0038) {
        switch (addr) {
            case REG_CONTROL_WORD:
                pending_control_word = value & CTRL_MODE_MASK;
                if ((pending_control_word & CTRL_MODE_MASK) > MODE_ADC_POSITION) {
                    pending_control_word = mc->mode;
                    break;
                }
                if (mc->start_mode == 0) {
                    MotorMode_t old_mode = mc->mode;
                    MODBUS_ApplyMode(ctx, pending_control_word & CTRL_MODE_MASK, 1);
                    if (mc->mode != old_mode) {
                        config_dirty = 1;
                    }
                }
                break;
            case REG_MODE_SET:
                mc->start_mode = value ? 1 : 0;
                start_flag_set = 0;
                config_dirty = 1;
                break;
            case REG_TARGET_POS_H3:
                pending_target_pos = (pending_target_pos & 0x0000FFFFFFFFFFFFLL) | ((int64_t)value << 48);
                if (!is_multiple && (mc->start_mode == 0 || start_flag_set)) need_apply = 1;
                break;
            case REG_TARGET_POS_H2:
                pending_target_pos = (pending_target_pos & 0xFFFF0000FFFFFFFFLL) | ((int64_t)value << 32);
                if (!is_multiple && (mc->start_mode == 0 || start_flag_set)) need_apply = 1;
                break;
            case REG_TARGET_POS_L2:
                pending_target_pos = (pending_target_pos & 0xFFFFFFFF0000FFFFLL) | ((int64_t)value << 16);
                if (!is_multiple && (mc->start_mode == 0 || start_flag_set)) need_apply = 1;
                break;
            case REG_TARGET_POS_L1:
                pending_target_pos = (pending_target_pos & 0xFFFFFFFFFFFF0000LL) | ((int64_t)value << 0);
                if (!is_multiple && (mc->start_mode == 0 || start_flag_set)) need_apply = 1;
                break;
            case REG_TARGET_SPEED_H:
                pending_target_speed = (pending_target_speed & 0x0000FFFF) | ((int32_t)value << 16);
                if (!is_multiple && (mc->start_mode == 0 || start_flag_set)) need_apply = 1;
                break;
            case REG_TARGET_SPEED_L:
                pending_target_speed = (pending_target_speed & 0xFFFF0000) | ((int32_t)value << 0);
                if (!is_multiple && (mc->start_mode == 0 || start_flag_set)) need_apply = 1;
                break;
            case REG_POS_KP:
            case REG_POS_KI:
            case REG_POS_KD:
                MotorControl_SetPosPID(mc,
                    (addr == REG_POS_KP) ? (float)(int16_t)value / 100.0f : mc->pos_Kp,
                    (addr == REG_POS_KI) ? (float)(int16_t)value / 100.0f : mc->pos_Ki,
                    (addr == REG_POS_KD) ? (float)(int16_t)value / 1000.0f : mc->pos_Kd);  /* D项×1000 */
                config_dirty = 1;
                break;
            case REG_SPD_KP:
            case REG_SPD_KI:
            case REG_SPD_KD:
                MotorControl_SetSpdPID(mc,
                    (addr == REG_SPD_KP) ? (float)(int16_t)value / 100.0f : mc->spd_Kp,
                    (addr == REG_SPD_KI) ? (float)(int16_t)value / 100.0f : mc->spd_Ki,
                    (addr == REG_SPD_KD) ? (float)(int16_t)value / 1000.0f : mc->spd_Kd);  /* D项×1000 */
                config_dirty = 1;
                break;
            case REG_DEAD_ZONE:
                mc->dead_zone = value;
                config_dirty = 1;
                break;
            case REG_MAX_OUTPUT:
                if (value < 10 || value > 1000) value = 900;
                mc->max_output = value;
                config_dirty = 1;
                break;
            case REG_START_MODE:
                if (mc->start_mode && value == 1) {
                    MotorMode_t old_mode = mc->mode;
                    start_flag_set = 1;
                    MODBUS_ApplyMode(ctx, pending_control_word & CTRL_MODE_MASK, 0);
                    if (mc->mode != old_mode) {
                        config_dirty = 1;
                    }
                } else if (value == 0) {
                    start_flag_set = 0;
                }
                break;
            case REG_DIRECTION:
                MotorControl_SetDirection(mc, (value != 0) ? MOTOR_DIR_CCW : MOTOR_DIR_CW);
                config_dirty = 1;
                break;
            case REG_ENCODER_DIRECTION:
                MotorControl_SetEncoderDirection(mc, (value != 0) ? ENCODER_DIR_REVERSED : ENCODER_DIR_NORMAL);
                config_dirty = 1;
                break;
            case REG_START_FLAG:
                if (value == 0) {
                    start_flag_set = 0;
                }
                break;
            case REG_SLAVE_ADDR:
                if (value >= 1 && value <= 247) {
                    ctx->slave_addr = (uint8_t)value;
                    mc->modbus_addr = (uint8_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_BAUD_RATE:
                pending_baud_rate = MODBUS_DecodeBaud(value);
                if (pending_baud_rate != 0) {
                    mc->modbus_baud = pending_baud_rate;
                    ctx->frame_timeout_ticks = MODBUS_GetFrameTimeoutTicks(pending_baud_rate);
                    config_dirty = 1;
                }
                break;
            case REG_SET_ORIGIN:
                if (value == 1) {
                    MotorControl_SetOrigin(mc);
                    pending_target_pos = 0;
                    pending_target_speed = 0;
                }
                break;
            case REG_HOME_MODE:
                if (value == 1) mc->home_mode = HOME_MODE_STALL;
                else if (value == 2) mc->home_mode = HOME_MODE_SWITCH;
                else mc->home_mode = HOME_MODE_OFF;
                config_dirty = 1;
                break;
            case REG_HOME_DIRECTION:
                mc->home_dir = (value != 0) ? 1 : 0;
                config_dirty = 1;
                break;
            case REG_HOME_CURRENT:
                if (value < 10 || value > 1000) value = 300;
                mc->home_current = (int16_t)value;
                config_dirty = 1;
                break;
            case REG_HOME_SPEED_H:
                mc->home_speed = (int32_t)(((uint32_t)value << 16) | ((uint32_t)mc->home_speed & 0xFFFF));
                config_dirty = 1;
                break;
            case REG_HOME_SPEED_L:
                mc->home_speed = (int32_t)(((uint32_t)mc->home_speed & 0xFFFF0000) | value);
                if (mc->home_speed <= 0) mc->home_speed = 1000;
                config_dirty = 1;
                break;
            case REG_HOME_MAX_DISTANCE_H:
                mc->home_max_distance = (int32_t)(((uint32_t)value << 16) | ((uint32_t)mc->home_max_distance & 0xFFFF));
                config_dirty = 1;
                break;
            case REG_HOME_MAX_DISTANCE_L:
                mc->home_max_distance = (int32_t)(((uint32_t)mc->home_max_distance & 0xFFFF0000) | value);
                if (mc->home_max_distance <= 0) mc->home_max_distance = 10000;
                config_dirty = 1;
                break;
            case REG_HOME_BACK_DISTANCE_H:
                mc->home_back_distance = (int32_t)(((uint32_t)value << 16) | ((uint32_t)mc->home_back_distance & 0xFFFF));
                config_dirty = 1;
                break;
            case REG_HOME_BACK_DISTANCE_L:
                mc->home_back_distance = (int32_t)(((uint32_t)mc->home_back_distance & 0xFFFF0000) | value);
                config_dirty = 1;
                break;
            case REG_HOME_TRIGGER:
                if (value == 1) {
                    MotorControl_StartHoming(mc);
                }
                break;
            case REG_HOME_AUTO_START:
                mc->home_auto_start = (value != 0) ? 1 : 0;
                config_dirty = 1;
                break;
            case REG_INPUT1_FUNC:
                if (value <= PIN_FUNC_START_FLAG) {
                    mc->pin4_func = (uint8_t)value;
                    config_dirty = 1;
                    MotorControl_UpdateIrqPriority();
                }
                break;
            case REG_INPUT1_POLARITY:
                mc->pin4_polarity = (value != 0) ? 1 : 0;
                config_dirty = 1;
                /* 极性变更后必须重新配置EXTI触发沿(上升沿↔下降沿) */
                if (mc->pin4_func == PIN_FUNC_PULSE) {
                    MotorControl_UpdateIrqPriority();
                }
                break;
            case REG_INPUT1_LIMIT_DIR:
                mc->pin4_limit_dir = (value != 0) ? 1 : 0;
                config_dirty = 1;
                break;
            case REG_INPUT2_FUNC:
                if (value <= PIN_FUNC_START_FLAG) {
                    mc->pin5_func = (uint8_t)value;
                    config_dirty = 1;
                    MotorControl_UpdateIrqPriority();
                }
                break;
            case REG_INPUT2_POLARITY:
                mc->pin5_polarity = (value != 0) ? 1 : 0;
                config_dirty = 1;
                /* 极性变更后必须重新配置EXTI触发沿(上升沿↔下降沿) */
                if (mc->pin5_func == PIN_FUNC_PULSE) {
                    MotorControl_UpdateIrqPriority();
                }
                break;
            case REG_INPUT2_LIMIT_DIR:
                mc->pin5_limit_dir = (value != 0) ? 1 : 0;
                config_dirty = 1;
                break;
            case REG_MAX_RUN_SPEED_H:
                mc->max_run_speed = (int32_t)(((uint32_t)value << 16) | ((uint32_t)mc->max_run_speed & 0xFFFF));
                config_dirty = 1;
                break;
            case REG_MAX_RUN_SPEED_L:
                mc->max_run_speed = (int32_t)(((uint32_t)mc->max_run_speed & 0xFFFF0000) | value);
                MotorControl_SetMaxRunSpeed(mc, mc->max_run_speed);
                config_dirty = 1;
                break;
            case REG_INPUT1_TARGET_POS_H3:
                mc->pin4_target_pos = (int64_t)(((uint64_t)mc->pin4_target_pos & 0x0000FFFFFFFFFFFFULL) | ((uint64_t)value << 48));
                config_dirty = 1;
                break;
            case REG_INPUT1_TARGET_POS_H2:
                mc->pin4_target_pos = (int64_t)(((uint64_t)mc->pin4_target_pos & 0xFFFF0000FFFFFFFFULL) | ((uint64_t)value << 32));
                config_dirty = 1;
                break;
            case REG_INPUT1_TARGET_POS_L2:
                mc->pin4_target_pos = (int64_t)(((uint64_t)mc->pin4_target_pos & 0xFFFFFFFF0000FFFFULL) | ((uint64_t)value << 16));
                config_dirty = 1;
                break;
            case REG_INPUT1_TARGET_POS_L1:
                mc->pin4_target_pos = (int64_t)(((uint64_t)mc->pin4_target_pos & 0xFFFFFFFFFFFF0000ULL) | (uint64_t)value);
                config_dirty = 1;
                break;
            case REG_INPUT2_TARGET_POS_H3:
                mc->pin5_target_pos = (int64_t)(((uint64_t)mc->pin5_target_pos & 0x0000FFFFFFFFFFFFULL) | ((uint64_t)value << 48));
                config_dirty = 1;
                break;
            case REG_INPUT2_TARGET_POS_H2:
                mc->pin5_target_pos = (int64_t)(((uint64_t)mc->pin5_target_pos & 0xFFFF0000FFFFFFFFULL) | ((uint64_t)value << 32));
                config_dirty = 1;
                break;
            case REG_INPUT2_TARGET_POS_L2:
                mc->pin5_target_pos = (int64_t)(((uint64_t)mc->pin5_target_pos & 0xFFFFFFFF0000FFFFULL) | ((uint64_t)value << 16));
                config_dirty = 1;
                break;
            case REG_INPUT2_TARGET_POS_L1:
                mc->pin5_target_pos = (int64_t)(((uint64_t)mc->pin5_target_pos & 0xFFFFFFFFFFFF0000ULL) | (uint64_t)value);
                config_dirty = 1;
                break;
            case REG_HOME_PRECISION_SPEED_H:
                mc->home_precision_speed = (int32_t)(((uint32_t)value << 16) | ((uint32_t)mc->home_precision_speed & 0xFFFF));
                config_dirty = 1;
                break;
            case REG_HOME_PRECISION_SPEED_L:
                mc->home_precision_speed = (int32_t)(((uint32_t)mc->home_precision_speed & 0xFFFF0000) | value);
                if (mc->home_precision_speed <= 0) mc->home_precision_speed = 100;
                config_dirty = 1;
                break;
            case REG_HOME_PRECISION_CYCLES:
                if (value >= 1 && value <= 20) {
                    mc->home_precision_cycles = (uint8_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_INPUT1_TARGET_SPEED_H:
                mc->pin4_target_speed = (int32_t)(((uint32_t)value << 16) | ((uint32_t)mc->pin4_target_speed & 0xFFFF));
                config_dirty = 1;
                break;
            case REG_INPUT1_TARGET_SPEED_L:
                mc->pin4_target_speed = (int32_t)(((uint32_t)mc->pin4_target_speed & 0xFFFF0000) | value);
                config_dirty = 1;
                break;
            case REG_INPUT2_TARGET_SPEED_H:
                mc->pin5_target_speed = (int32_t)(((uint32_t)value << 16) | ((uint32_t)mc->pin5_target_speed & 0xFFFF));
                config_dirty = 1;
                break;
            case REG_INPUT2_TARGET_SPEED_L:
                mc->pin5_target_speed = (int32_t)(((uint32_t)mc->pin5_target_speed & 0xFFFF0000) | value);
                config_dirty = 1;
                break;
        }
    }

    /* 转速采集控制寄存器 0x003A~0x003F */
    if (addr >= REG_SPEED_ACQ_START && addr <= REG_SPEED_ACQ_SIZE) {
        switch (addr) {
            case REG_SPEED_ACQ_START:
                if (value == 1) {
                    /* 启动采集：清空缓冲区并激活 */
                    mc->speed_acq_count = 0;
                    mc->speed_acq_div_cnt = 0;
                    mc->speed_acq_active = 1;
                    mc->speed_acq_done = 0;
                    /* 位置采样(type=2)启动时记录当前位置作为基准 */
                    if (mc->speed_acq_type == 2) {
                        mc->speed_acq_pos_start = Encoder_GetCount();
                    }
                } else if (value == 0) {
                    /* 停止采集 */
                    mc->speed_acq_active = 0;
                }
                break;
            case REG_SPEED_ACQ_DIV:
                if (value >= 1) {
                    mc->speed_acq_divider = value;
                }
                break;
            case REG_SPEED_ACQ_TYPE:
                if (value <= 6) {
                    mc->speed_acq_type = (uint8_t)value;
                }
                break;
            case REG_SPEED_ACQ_SIZE:
                if (value >= 1 && value <= SPEED_ACQ_BUF_SIZE) {
                    mc->speed_acq_size = value;
                }
                break;
            case REG_SPEED_ACQ_COUNT:
            case REG_SPEED_ACQ_STATUS:
                /* 只读寄存器，忽略写入 */
                break;
        }
    }

    /* 堵转保护控制寄存器 0x0040~0x0044 */
    if (addr >= REG_STALL_PROT_EN && addr <= REG_STALL_RESET) {
        switch (addr) {
            case REG_STALL_PROT_EN:
                mc->stall_protect_en = value ? 1 : 0;
                config_dirty = 1;
                break;
            case REG_STALL_ERR_LIMIT:
                mc->stall_err_limit = (int32_t)(int16_t)value;  /* 允许负值(无意义但兼容) */
                config_dirty = 1;
                break;
            case REG_STALL_TIME:
                mc->stall_time_ticks = value;
                config_dirty = 1;
                break;
            case REG_STALL_RESET:
                if (value == 1) {
                    MotorControl_ResetStall(mc);
                }
                break;
            case REG_STALL_STATUS:
                /* 只读寄存器，忽略写入 */
                break;
        }
    }

    /* 电流环控制寄存器 0x0045~0x004D (新增) */
    if (addr >= REG_CUR_LOOP_EN && addr <= REG_OVER_CUR_RESET) {
        switch (addr) {
            case REG_CUR_LOOP_EN:
                MotorControl_EnableCurrentLoop(mc, value ? 1 : 0);
                config_dirty = 1;
                break;
            case REG_CUR_KP:
            case REG_CUR_KI:
            case REG_CUR_KD: {
                float Kp = mc->cur_Kp;
                float Ki = mc->cur_Ki;
                float Kd = mc->cur_Kd;
                if (addr == REG_CUR_KP) Kp = (float)(int16_t)value / 100.0f;
                if (addr == REG_CUR_KI) Ki = (float)(int16_t)value / 100.0f;
                if (addr == REG_CUR_KD) Kd = (float)(int16_t)value / 1000.0f;
                MotorControl_SetCurPID(mc, Kp, Ki, Kd);
                config_dirty = 1;
                break;
            }
            case REG_CUR_OFFSET:
                if (value <= 4095) {
                    MotorControl_SetCurrentCalib(mc, value, mc->current_scale);
                    config_dirty = 1;
                }
                break;
            case REG_CUR_SCALE:
                if (value > 0) {
                    MotorControl_SetCurrentCalib(mc, mc->current_offset, (float)value / 10000.0f);
                    config_dirty = 1;
                }
                break;
            case REG_OVER_CUR_LIMIT:
                MotorControl_SetOverCurrentLimit(mc, (float)(int16_t)value / 10.0f);
                config_dirty = 1;
                break;
            case REG_OVER_CUR_RESET:
                if (value == 1) {
                    MotorControl_ResetOverCurrent(mc);
                }
                break;
            case REG_OVER_CUR_STATUS:
                /* 只读寄存器，忽略写入 */
                break;
        }
    }

    /* PC2/PC3 ADC功能寄存器 0x004E~0x0061 (新增双死区) */
    if (addr >= REG_PC2_FUNC && addr <= REG_ADC_DEAD_ZONE2_WIDTH) {
        switch (addr) {
            case REG_PC2_FUNC:
                if (value <= ADC_FUNC_POSITION) {
                    mc->pc2_func = (uint8_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_PC3_FUNC:
                if (value <= ADC_FUNC_POSITION) {
                    mc->pc3_func = (uint8_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_ADC_MAX_SPEED_H:
                mc->adc_max_speed = (mc->adc_max_speed & 0x0000FFFF) | ((int32_t)(int16_t)value << 16);
                config_dirty = 1;
                break;
            case REG_ADC_MAX_SPEED_L:
                mc->adc_max_speed = (mc->adc_max_speed & 0xFFFF0000) | (int32_t)value;
                config_dirty = 1;
                break;
            case REG_ADC_MAX_PWM: {
                int16_t pwm = (int16_t)value;
                if (pwm > 1000) pwm = 1000;
                if (pwm < -1000) pwm = -1000;
                mc->adc_max_pwm = pwm;
                config_dirty = 1;
                break;
            }
            case REG_ADC_MAX_POS_H3:
            case REG_ADC_MAX_POS_H2:
            case REG_ADC_MAX_POS_L2:
            case REG_ADC_MAX_POS_L1: {
                /* 64位位置分4个16位寄存器写入, 使用掩码拼接 */
                uint64_t mask = 0xFFFFULL;
                uint8_t shift = 0;
                switch (addr) {
                    case REG_ADC_MAX_POS_H3: shift = 48; break;
                    case REG_ADC_MAX_POS_H2: shift = 32; break;
                    case REG_ADC_MAX_POS_L2: shift = 16; break;
                    case REG_ADC_MAX_POS_L1: shift = 0;  break;
                }
                mc->adc_max_position = (int64_t)((((uint64_t)mc->adc_max_position) & ~(mask << shift)) |
                                                  ((uint64_t)value << shift));
                config_dirty = 1;
                break;
            }
            /* ADC最小值寄存器写入(新增) */
            case REG_ADC_MIN_SPEED_H:
                mc->adc_min_speed = (mc->adc_min_speed & 0x0000FFFF) | ((int32_t)(int16_t)value << 16);
                config_dirty = 1;
                break;
            case REG_ADC_MIN_SPEED_L:
                mc->adc_min_speed = (mc->adc_min_speed & 0xFFFF0000) | (int32_t)value;
                config_dirty = 1;
                break;
            case REG_ADC_MIN_PWM: {
                int16_t pwm = (int16_t)value;
                if (pwm > 1000) pwm = 1000;
                if (pwm < -1000) pwm = -1000;
                mc->adc_min_pwm = pwm;
                config_dirty = 1;
                break;
            }
            case REG_ADC_MIN_POS_H3:
            case REG_ADC_MIN_POS_H2:
            case REG_ADC_MIN_POS_L2:
            case REG_ADC_MIN_POS_L1: {
                uint64_t mask = 0xFFFFULL;
                uint8_t shift = 0;
                switch (addr) {
                    case REG_ADC_MIN_POS_H3: shift = 48; break;
                    case REG_ADC_MIN_POS_H2: shift = 32; break;
                    case REG_ADC_MIN_POS_L2: shift = 16; break;
                    case REG_ADC_MIN_POS_L1: shift = 0;  break;
                }
                mc->adc_min_position = (int64_t)((((uint64_t)mc->adc_min_position) & ~(mask << shift)) |
                                                  ((uint64_t)value << shift));
                config_dirty = 1;
                break;
            }
            /* ADC死区寄存器写入(新增, 双死区) */
            case REG_ADC_DEAD_ZONE1_POS:
                if (value <= 2) {
                    mc->adc_dead_zone1_pos = (uint8_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_ADC_DEAD_ZONE1_WIDTH:
                if (value <= 4095) {
                    mc->adc_dead_zone1_width = (uint16_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_ADC_DEAD_ZONE2_POS:
                if (value <= 2) {
                    mc->adc_dead_zone2_pos = (uint8_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_ADC_DEAD_ZONE2_WIDTH:
                if (value <= 4095) {
                    mc->adc_dead_zone2_width = (uint16_t)value;
                    config_dirty = 1;
                }
                break;
        }
    }

    /* 如果需要立即应用目标值 */
    if (need_apply) {
        MODBUS_ApplyTarget(ctx);
    }
}

/**
  * @brief  处理功能码03（读保持寄存器）
  */
static void MODBUS_ProcessReadHoldingRegs(MB_Context_t *ctx, uint16_t start_addr, uint16_t quantity)
{
    /* 检查地址范围 */
    if (quantity > 125) {
        MODBUS_SendException(ctx, MB_FUNC_READ_HOLDING_REGS, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    /* 检查地址范围：控制寄存器 0x0000~0x0061, 状态寄存器 0x0100~0x0115, 数据寄存器 0x0200~0x15FF */
    uint16_t end_addr = start_addr + quantity - 1;
    if (!((start_addr <= 0x0061 && end_addr <= 0x0061) ||
          (start_addr >= 0x0100 && end_addr <= 0x0115) ||
          (start_addr >= REG_SPEED_DATA_BASE && end_addr <= REG_SPEED_DATA_END))) {
        MODBUS_SendException(ctx, MB_FUNC_READ_HOLDING_REGS, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    uint8_t response[MB_TX_BUF_SIZE];
    uint8_t idx = 0;
    
    response[idx++] = ctx->slave_addr;
    response[idx++] = MB_FUNC_READ_HOLDING_REGS;
    response[idx++] = quantity * 2;  /* 字节数 */
    
    /* 读取寄存器 */
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t value = MODBUS_ReadRegister(ctx, start_addr + i);
        response[idx++] = (value >> 8) & 0xFF;  /* 高字节 */
        response[idx++] = value & 0xFF;          /* 低字节 */
    }
    
    MODBUS_SendResponse(ctx, response, idx);
}

/**
  * @brief  处理功能码06（写单个寄存器）
  */
static void MODBUS_ProcessWriteSingleReg(MB_Context_t *ctx, uint16_t addr, uint16_t value)
{
    uint8_t old_slave_addr = ctx->slave_addr;

    /* 检查地址范围 (控制寄存器 0x0000~0x0061 可写, 状态/数据寄存器只读) */
    if (addr > 0x0061) {
        MODBUS_SendException(ctx, MB_FUNC_WRITE_SINGLE_REG, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    /* 写入寄存器（单寄存器写入，is_multiple=0） */
    MODBUS_WriteRegister(ctx, addr, value, 0);
    
    /* 响应：原样返回 */
    uint8_t response[6];
    response[0] = (addr == REG_SLAVE_ADDR) ? old_slave_addr : ctx->slave_addr;
    response[1] = MB_FUNC_WRITE_SINGLE_REG;
    response[2] = (addr >> 8) & 0xFF;
    response[3] = addr & 0xFF;
    response[4] = (value >> 8) & 0xFF;
    response[5] = value & 0xFF;
    
    MODBUS_SendResponse(ctx, response, 6);
}

/**
  * @brief  处理功能码10（写多个寄存器）
  */
static void MODBUS_ProcessWriteMultipleRegs(MB_Context_t *ctx, uint16_t start_addr, uint16_t quantity, uint8_t *data)
{
    MotorControl_t *mc = ctx->motor;
    uint8_t old_slave_addr = ctx->slave_addr;
    pending_baud_rate = 0;
    /* 检查地址范围 (控制寄存器 0x0000~0x0061 可写) */
    if (quantity > 125 || start_addr + quantity > 0x0062) {
        MODBUS_SendException(ctx, MB_FUNC_WRITE_MULTIPLE_REGS, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    /* 写入寄存器（多寄存器写入，is_multiple=1，不立即启动） */
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t value = (data[i * 2] << 8) | data[i * 2 + 1];
        MODBUS_WriteRegister(ctx, start_addr + i, value, 1);
    }
    
    /* 多寄存器写入完成后，如果写入了目标位置/速度，则应用目标值 */
    {
        /* 检查是否写入了目标位置或速度寄存器 */
        uint8_t wrote_target = 0;
        if (start_addr <= REG_TARGET_POS_L1 && start_addr + quantity > REG_TARGET_POS_H3) {
            wrote_target = 1;  /* 写入了目标位置 */
        }
        if (start_addr <= REG_TARGET_SPEED_L && start_addr + quantity > REG_TARGET_SPEED_H) {
            wrote_target = 1;  /* 写入了目标速度 */
        }
        if (wrote_target && (mc->start_mode == 0 || start_flag_set)) {
            MODBUS_ApplyTarget(ctx);
        }
    }
    
    /* 响应 */
    uint8_t response[6];
    response[0] = (start_addr <= REG_SLAVE_ADDR && start_addr + quantity > REG_SLAVE_ADDR) ? old_slave_addr : ctx->slave_addr;
    response[1] = MB_FUNC_WRITE_MULTIPLE_REGS;
    response[2] = (start_addr >> 8) & 0xFF;
    response[3] = start_addr & 0xFF;
    response[4] = (quantity >> 8) & 0xFF;
    response[5] = quantity & 0xFF;
    
    MODBUS_SendResponse(ctx, response, 6);
}

/**
  * @brief  处理MODBUS帧
  */
void MODBUS_Process(MB_Context_t *ctx)
{
    /* 检查是否有完整帧 */
    if (!ctx->rx_complete) {
        return;
    }
    
    pending_baud_rate = 0;

    /* 检查最小帧长度 */
    if (ctx->rx_len < 4) {
        ctx->rx_complete = 0;
        ctx->rx_len = 0;
        return;
    }
    
    /* 检查从机地址 - 使用rx_process_buf */
    if (ctx->rx_process_buf[0] != ctx->slave_addr && ctx->rx_process_buf[0] != 0x00) {
        ctx->rx_complete = 0;
        ctx->rx_len = 0;
        return;
    }
    
    /* 验证CRC - 使用rx_process_buf */
    uint16_t recv_crc = (ctx->rx_process_buf[ctx->rx_len - 1] << 8) | ctx->rx_process_buf[ctx->rx_len - 2];
    uint16_t calc_crc = MODBUS_CalcCRC(ctx->rx_process_buf, ctx->rx_len - 2);
    if (recv_crc != calc_crc) {
        ctx->rx_complete = 0;
        ctx->rx_len = 0;
        ctx->error_count++;
        return;
    }
    
    /* 解析帧 - 使用rx_process_buf */
    uint8_t func = ctx->rx_process_buf[1];
    
    switch (func) {
        case MB_FUNC_READ_HOLDING_REGS: {
            uint16_t start_addr = (ctx->rx_process_buf[2] << 8) | ctx->rx_process_buf[3];
            uint16_t quantity = (ctx->rx_process_buf[4] << 8) | ctx->rx_process_buf[5];
            MODBUS_ProcessReadHoldingRegs(ctx, start_addr, quantity);
            break;
        }
        
        case MB_FUNC_WRITE_SINGLE_REG: {
            uint16_t addr = (ctx->rx_process_buf[2] << 8) | ctx->rx_process_buf[3];
            uint16_t value = (ctx->rx_process_buf[4] << 8) | ctx->rx_process_buf[5];
            MODBUS_ProcessWriteSingleReg(ctx, addr, value);
            break;
        }
        
        case MB_FUNC_WRITE_MULTIPLE_REGS: {
            uint16_t start_addr = (ctx->rx_process_buf[2] << 8) | ctx->rx_process_buf[3];
            uint16_t quantity = (ctx->rx_process_buf[4] << 8) | ctx->rx_process_buf[5];
            MODBUS_ProcessWriteMultipleRegs(ctx, start_addr, quantity, &ctx->rx_process_buf[7]);
            break;
        }
        
        default:
            MODBUS_SendException(ctx, func, MB_EX_ILLEGAL_FUNCTION);
            break;
    }
    
    /* 清空接收标志 */
    ctx->rx_complete = 0;
    ctx->rx_len = 0;

    /* 如果本次帧修改了需要保存的参数，统一写入Flash（每帧最多擦写一次） */
    if (config_dirty) {
        MotorControl_SaveConfig(ctx->motor);
        config_dirty = 0;
    }

    if (pending_baud_rate != 0) {
        MODBUS_ApplyBaudRate(ctx, pending_baud_rate);
        pending_baud_rate = 0;
    }
}
