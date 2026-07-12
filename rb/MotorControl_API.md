# 电机控制器 API 文档

## 硬件连接

| 外设 | 引脚 | 功能 |
|---|---|---|
| **TIM4_CH1** | PB6 | H桥正向PWM (0~max_output) |
| **TIM4_CH2** | PB7 | 使能1 (始终100%) |
| **TIM4_CH3** | PB8 | H桥反向PWM (0~max_output) |
| **TIM4_CH4** | PB9 | 使能2 (始终100%) |
| **TIM3_CH1** | PA6 | 编码器A相 |
| **TIM3_CH2** | PA7 | 编码器B相 |
| **TIM2** | — | 5ms定时器，驱动PID运算周期 |

## 中断依赖

| 中断 | 用途 | 实现位置 |
|---|---|---|
| `TIM2_IRQHandler` | 5ms触发→HAL_TIM_PeriodElapsedCallback→PID运算 | **motor_control.c** (库自带) |
| `TIM3_IRQHandler` | 编码器溢出→Encoder_OverflowHandler()→64位扩展 | **stm32f1xx_it.c** (用户实现) |

### stm32f1xx_it.c 中需要添加的代码

```c
/* Includes */
#include "motor_control.h"

/* TIM3中断处理 */
void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        Encoder_OverflowHandler();
    }
    HAL_TIM_IRQHandler(&htim3);
}
```

## main.c 初始化顺序

```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();      /* 编码器 */
    MX_TIM4_Init();      /* PWM */
    MX_TIM2_Init();      /* 5ms定时 */

    MotorControl_Init(&motor, 1.0f, 0.01f, 0.1f, 0.5f, 0.05f, 0.01f, 10, 900);
    Motor_Start();
    HAL_TIM_Base_Start_IT(&htim2);

    while (1) {
        /* 用户代码 */
    }
}
```

---

## API 总览

### 模式

```c
POSITION_MODE   /* 位置模式：控制电机转到目标位置 */
SPEED_MODE      /* 速度模式：控制电机以目标速度运转 */
```

### 初始化与启停

| 函数 | 说明 |
|---|---|
| `MotorControl_Init(&motor, pos_Kp, pos_Ki, pos_Kd, spd_Kp, spd_Ki, spd_Kd, dead_zone, max_output)` | 初始化位置PID、速度PID、死区、最大PWM |
| `Motor_Start()` | 启动PWM输出、编码器计数、使能TIM3溢出中断 |
| `Motor_Stop()` | 停止PWM和编码器，断电 |

### 参数设置

| 函数 | 说明 |
|---|---|
| `MotorControl_SetPosPID(&motor, Kp, Ki, Kd)` | 调整**位置模式**PID参数 |
| `MotorControl_SetSpdPID(&motor, Kp, Ki, Kd)` | 调整**速度模式**PID参数 |
| `MotorControl_SetMaxOutput(&motor, max)` | 限制最大PWM(0~1000)，控制电机电流 |
| `MotorControl_SetMode(&motor, mode)` | 切换位置/速度模式 |
| `MotorControl_SetTarget(&motor, pos)` | 位置模式：设置目标位置(脉冲数, int64_t) |
| `MotorControl_SetSpeed(&motor, speed)` | 速度模式：设置目标速度(脉冲/秒) |
| `MotorControl_SetOrigin(&motor)` | 将当前位置设为原点：编码器清零+目标位置/积分/速度滤波同步清零（TIM2中断临界区保护） |

### 数据读取

| 函数 | 返回值 | 说明 |
|---|---|---|
| `Encoder_GetCount()` | `int64_t` | 编码器当前位置，溢出自动扩展 |
| `Encoder_GetSpeed()` | `float` | 当前速度(脉冲/秒)，每5ms更新 |
| `MotorControl_GetTarget(&motor)` | `int64_t` | 位置模式目标值 |
| `MotorControl_GetSpeedTarget(&motor)` | `float` | 速度模式目标值 |
| `MotorControl_GetMode(&motor)` | `MotorMode_t` | 当前模式 |

### 其他

| 函数 | 说明 |
|---|---|
| `Encoder_Reset()` | 编码器当前位置归零 |
| `Encoder_OverflowHandler()` | TIM3溢出时调用，由TIM3_IRQHandler触发 |
| `MotorControl_Reset(&motor)` | 复位PID积分项，PWM置零 |

---

## 使用示例

### 示例1：位置模式——在两位置间往复

```c
while (1)
{
    MotorControl_SetTarget(&motor, 5000);   /* 转到位置5000 */
    HAL_Delay(2000);

    MotorControl_SetTarget(&motor, 0);      /* 回到位置0 */
    HAL_Delay(2000);
}
```

### 示例2：位置模式——串口控制位置

```c
while (1)
{
    if (HAL_UART_Receive(&huart1, (uint8_t*)&cmd, 4, 100) == HAL_OK)
    {
        MotorControl_SetTarget(&motor, (int64_t)cmd);
    }
    HAL_Delay(10);
}
```

### 示例3：速度模式

```c
MotorControl_SetMode(&motor, SPEED_MODE);
MotorControl_SetSpeed(&motor, 800.0f);      /* 正转800脉冲/秒 */
HAL_Delay(3000);

MotorControl_SetSpeed(&motor, -500.0f);     /* 反转500脉冲/秒 */
HAL_Delay(3000);

MotorControl_SetSpeed(&motor, 0);           /* 停止 */
```

### 示例4：限流保护

```c
/* 启动时限制电流，防止过冲 */
MotorControl_SetMaxOutput(&motor, 300);     /* PWM最大30% */
MotorControl_SetTarget(&motor, 10000);
HAL_Delay(500);

/* 正常运行放开限制 */
MotorControl_SetMaxOutput(&motor, 900);     /* PWM最大90% */
```

### 示例5：实时调参

```c
while (1)
{
    if (load_heavy) {
        MotorControl_SetPosPID(&motor, 2.0f, 0.05f, 0.3f);
        MotorControl_SetSpdPID(&motor, 0.8f, 0.1f, 0.02f);
    } else {
        MotorControl_SetPosPID(&motor, 1.0f, 0.01f, 0.1f);
        MotorControl_SetSpdPID(&motor, 0.5f, 0.05f, 0.01f);
    }

    int64_t pos = Encoder_GetCount();
    float   vel = Encoder_GetSpeed();
    int64_t err = MotorControl_GetTarget(&motor) - pos;

    HAL_Delay(100);
}
```

### 示例6：回零

方式一：将当前位置设为原点（电机保持在当前位置，坐标归零）：

```c
MotorControl_SetOrigin(&motor);   /* 编码器清零，目标位置同步为0 */
```

方式二：回到机械原点（编码器复位到0并移动到位置0）：

```c
Encoder_Reset();
MotorControl_SetTarget(&motor, 0);
```
