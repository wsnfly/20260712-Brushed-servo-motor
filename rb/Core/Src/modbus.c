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
    if (addr >= 0x0100 && addr <= 0x0108) {
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
            case REG_STATUS:
                return (start_flag_set ? 0x8000 : 0) |
                       (MotorControl_IsHoming() ? 0x4000 : 0) |
                       (MotorControl_HomingFailed() ? 0x2000 : 0) |
                       (mc->mode & 0x0F);
            case REG_CURRENT_PWM:
                return (uint16_t)MotorControl_GetPWM();
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
                return (uint16_t)(int16_t)(mc->pos_Kd * 100);
            case REG_SPD_KP:
                return (uint16_t)(int16_t)(mc->spd_Kp * 100);
            case REG_SPD_KI:
                return (uint16_t)(int16_t)(mc->spd_Ki * 100);
            case REG_SPD_KD:
                return (uint16_t)(int16_t)(mc->spd_Kd * 100);
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
    if (addr >= 0x0100 && addr <= 0x0107) {
        return;
    }
    
    /* 控制寄存器 */
    if (addr >= 0x0000 && addr <= 0x0038) {
        switch (addr) {
            case REG_CONTROL_WORD:
                pending_control_word = value & CTRL_MODE_MASK;
                if ((pending_control_word & CTRL_MODE_MASK) > MODE_EXTERNAL_TARGET_SPEED) {
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
                    (addr == REG_POS_KD) ? (float)(int16_t)value / 100.0f : mc->pos_Kd);
                config_dirty = 1;
                break;
            case REG_SPD_KP:
            case REG_SPD_KI:
            case REG_SPD_KD:
                MotorControl_SetSpdPID(mc,
                    (addr == REG_SPD_KP) ? (float)(int16_t)value / 100.0f : mc->spd_Kp,
                    (addr == REG_SPD_KI) ? (float)(int16_t)value / 100.0f : mc->spd_Ki,
                    (addr == REG_SPD_KD) ? (float)(int16_t)value / 100.0f : mc->spd_Kd);
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
                if (value <= PIN_FUNC_TARGET) {
                    mc->pin4_func = (uint8_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_INPUT1_POLARITY:
                mc->pin4_polarity = (value != 0) ? 1 : 0;
                config_dirty = 1;
                break;
            case REG_INPUT1_LIMIT_DIR:
                mc->pin4_limit_dir = (value != 0) ? 1 : 0;
                config_dirty = 1;
                break;
            case REG_INPUT2_FUNC:
                if (value <= PIN_FUNC_TARGET) {
                    mc->pin5_func = (uint8_t)value;
                    config_dirty = 1;
                }
                break;
            case REG_INPUT2_POLARITY:
                mc->pin5_polarity = (value != 0) ? 1 : 0;
                config_dirty = 1;
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
    /* 检查地址范围：控制寄存器 0x0000~0x0038, 状态寄存器 0x0100~0x0108 */
    uint16_t end_addr = start_addr + quantity - 1;
    if (!((start_addr <= 0x0038 && end_addr <= 0x0038) ||
          (start_addr >= 0x0100 && end_addr <= 0x0108))) {
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

    /* 检查地址范围 */
    if (addr > 0x0038) {
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
    /* 检查地址范围 */
    if (quantity > 125 || start_addr + quantity > 0x0039) {
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
