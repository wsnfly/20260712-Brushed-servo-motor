/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "modbus.h"
#include "input.h"      /* MotorControl_ProcessInputs / MotorControl_UpdateIrqPriority */
#include "afangkaobei.h"
#include "ajiajiansu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
MotorControl_t motor;
MB_Context_t mb_ctx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化电机控读取PID: 位置�???? Kp=7.00 Ki=10.00 Kd=0.10 | 速度�???? Kp=0.30 Ki=3.00 Kd=0.00�????? */
  MotorControl_Init(&motor, 7.0f, 10.1f, 0.1f, 0.30f, 3.0f, 0.0f, 1, 900);
  if (MODBUS_IsSupportedBaud(motor.modbus_baud) && huart1.Init.BaudRate != motor.modbus_baud) {
    HAL_UART_DeInit(&huart1);
    huart1.Init.BaudRate = motor.modbus_baud;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
      Error_Handler();
    }
  }

  /* 初始化MODBUS RTU协议 */
  MODBUS_Init(&mb_ctx, motor.modbus_addr, &motor);

  /* 启动电机控制 */
  Motor_Start();

  /* 加速表开机加载(新增): 检查Flash中是否有自定义加速表, 有则切换指针 */
  AccTable_LoadOnBoot();
  /* 同步加速表参数到MODBUS缓存: 使上位机读取0x0092~0x0097时能读到Flash中的实际参数,
   * 而非默认值. 解决重启后上位机显示默认参数、用户误以为表被重置的问题 */
  MODBUS_SyncAccParams();

  if (motor.home_auto_start) {
    MotorControl_StartHoming(&motor);
  }
  /* 启动TIM1 100us中断（MODBUS帧间超时 + PID定时器滴答 */
  HAL_TIM_Base_Start_IT(&htim1);
  /* TIM1固定优先级1(覆盖CubeMX在MspInit中的默认值2, 不再随脉冲模式动态切换)
   * EXTI脉冲中断同为优先级1, 同级不互相抢占, 靠硬件PR锁存保证不丢脉冲 */
  //HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);

  /* 启动UART中断接收（每字节，由HAL_UART_RxCpltCallback处理） */
  HAL_UART_Receive_IT(&huart1, &mb_ctx.rx_byte, 1);

  /* EXTI外部中断由MotorControl_UpdateIrqPriority统一配置:
   *   PB4/PB5配置为PIN_FUNC_PULSE时, 启用EXTI边沿触发(优先级1, 与TIM1同级)
   *   无脉冲输入时关闭EXTI NVIC, 释放CPU给USART1等
   * 注: TIM2已释放可作其他用途, 由EXTI4/EXTI9_5中断直接处理脉冲边沿 */
  MotorControl_UpdateIrqPriority();

  /* 中断优先级配置(CubeMX重新生成后需重新设置, 否则会被MspInit的默认值覆盖)
   * 优先级原则(数值小=优先级高):
   *   USART1=0 (最高) - 串口字节到达必须立即处理, 避免ORE溢出丢包
   *   TIM1_UP=1       - 100μs周期PID, 被USART1短时抢占(<5μs)不影响控制
   *   EXTI4/EXTI9_5=1 - 脉冲输入, 与TIM1同级不互相抢占, 靠硬件PR锁存保证不丢边沿
   *                    不能设为0(高于TIM1), 否则高频脉冲会频繁打断PID导致周期抖动
   * 注: MX_GPIO_Init设EXTI=0(最高), 此处覆盖为1(与TIM1同级), 防止脉冲中断抢占PID
   * 注: stm32f1xx_hal_msp.c中HAL MSP init设TIM1=2/USART1=3, 此处覆盖为正确值 */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);   /* TIM2步进加减速: 最低优先级, 不抢PID和通信 */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 步进加减速: 若Flash中使能(上位机勾选并保存), 初始化算法并启动TIM2 (新增)
   * 未使能时TIM2不启动, 保持原有行为不变 */
  if (motor.step_accel_en) {
      StepAccel_Enable(1, Encoder_GetCount());
  }
  while(fangkaobei(0) == 1);
  while (1)
  {
	  // 示例：每 500ms 执行�????�????
	  static int64_t last_500ms = 0;
	  int64_t now = SYS_GetTimeMs();
	  if (now - last_500ms >= 500) {
		  last_500ms = now;
	  }

	  /* ========== LED1 状态指示灯 ==========
	   * 正常状态：常亮（高电平）
	   * 异常状态（复位中/复位失败/堵转/过流）：闪烁（200ms周期，约5Hz） */
	  {
		  uint8_t has_fault = 0;
		  if (MotorControl_IsHoming())           has_fault = 1;  /* 正在复位 */
		  else if (MotorControl_HomingFailed())  has_fault = 1;  /* 复位失败 */
		  else if (MotorControl_IsStallTripped()) has_fault = 1; /* 堵转保护 */
		  else if (motor.over_current_tripped)   has_fault = 1;  /* 过流保护 */

		  if (has_fault) {
			  static int64_t last_led1_toggle = 0;
			  if (now - last_led1_toggle >= 200) {
				  last_led1_toggle = now;
				  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			  }
		  } else {
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);  /* 常亮（高电平） */
		  }
	  }

	  /* ========== LED2 通讯状态指示灯 ==========
	   * 收到MODBUS帧时点亮50ms，平时熄灭 */
	  {
		  static uint32_t last_rx_count = 0;
		  static int64_t led2_on_time = 0;
		  if (mb_ctx.rx_count != last_rx_count) {
			  last_rx_count = mb_ctx.rx_count;
			  led2_on_time = now;
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		  } else if (now - led2_on_time >= 50) {
			  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		  }
	  }

	  // 示例：每 2 秒执行一�????
	  static int64_t last_2s = 0;
	  if (now - last_2s >= 5000) {
		  last_2s = now;
		  // 你的逻辑

	  }

	  /* ========== 步进加减速位置模式 ==========
	   * dangqianweizhi → target_position 的同步已移至 TIM1 中断(MotorControl_TimerTick100us),
	   * 在位置PID(5ms)运行前执行, 确保时序同步, 避免异步复制导致的转速抖动.
	   * 支持所有基于位置PID的模式: POSITION_MODE/VELOCITY_POSITION_MODE/EXTERNAL_TARGET_MODE/ADC_POSITION_MODE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ============ 用户示例代码（注释形式，取消注释即可使用）============ */

    /* 示例1：旋转到指定位置（绝对位置，单位=编码器脉冲）
     * 注意1：SetMode 切到位置模式时会把当前编码器值作为目标，
     *        所以必须紧接着 SetTarget 设真正的目标，否则电机不动
     * 注意2：上位机读的目标位置是 MODBUS 内部的 pending 缓存，不是 mc->target_position。
     *        主循环里改了目标位置后，必须再调 MODBUS_SyncTargetPosition 同步，上位机才能读到 */
    // MotorControl_SetMode(&motor, POSITION_MODE);
    // MotorControl_SetTarget(&motor, 10000);          // 转到 10000 脉冲位置
    // MODBUS_SyncTargetPosition(10000);               // 同步到 MODBUS pending 缓存，上位机才能读到
    // Motor_Start();

    /* 示例2：以指定速度旋转（单位=脉冲/秒，负值=反转）*/
    // MotorControl_SetMode(&motor, SPEED_MODE);
    // MotorControl_SetSpeed(&motor, 2000.0f);  // 2000 脉冲/秒
    // Motor_Start();

    /* 示例3：读当前位置/速度，可用于条件判断 */
    // int64_t cur_pos = Encoder_GetCount();    // 当前位置（脉冲）
    // float    cur_spd = Encoder_GetSpeed();    // 当前速度（脉冲/秒）
    // int16_t  cur_pwm = MotorControl_GetPWM(); // 当前PWM输出(-1000~+1000)

    /* 示例4：顺序动作状态机——先到5000再到-5000，到位停1秒
     * 放在主循环每次都执行的位置（非节拍块内），用 now 做时间判断 */
    // static uint8_t  run_step = 0;
    // static int64_t  run_time = 0;
    // switch (run_step) {
    //     case 0:
    //         MotorControl_SetMode(&motor, POSITION_MODE);
    //         MotorControl_SetTarget(&motor, 5000);
    //         MODBUS_SyncTargetPosition(5000);     // 同步pending，上位机可见
    //         run_step = 1; run_time = now;
    //         break;
    //     case 1:
    //         if (llabs(Encoder_GetCount() - 5000) < 20 && (now - run_time) > 1000) {
    //             MotorControl_SetTarget(&motor, -5000);
    //             MODBUS_SyncTargetPosition(-5000); // 同步pending，上位机可见
    //             run_step = 2; run_time = now;
    //         }
    //         break;
    //     case 2:
    //         if (llabs(Encoder_GetCount() + 5000) < 20) {
    //             Motor_Stop();
    //             run_step = 3;
    //         }
    //         break;
    //     default: break;
    // }

    /* 示例5：每500ms切换速度方向做正反往复——应放在上面 last_500ms 节拍块内 */
    // static uint8_t toggle = 0;
    // MotorControl_SetMode(&motor, SPEED_MODE);
    // MotorControl_SetSpeed(&motor, toggle ? 1500.0f : -1500.0f);
    // toggle ^= 1;

    /* 示例6：每3秒在 0 和 10000 之间来回摆动（位置模式）*/
    // static uint8_t  pos_toggle = 0;
    // static int64_t  last_move  = 0;
    // if (now - last_move > 3000) {
    //     last_move = now;
    //     MotorControl_SetMode(&motor, POSITION_MODE);
    //     int64_t tgt = pos_toggle ? 0 : 10000;
    //     MotorControl_SetTarget(&motor, tgt);
    //     MODBUS_SyncTargetPosition(tgt);          // 同步pending，上位机可见
    //     pos_toggle ^= 1;
    // }

    /* 示例7：开机先回零，完成后转到 8000 脉冲处 */
    // if (!MotorControl_IsHoming()) {
    //     if (MotorControl_HomingFailed()) {
    //         // 复位失败处理……
    //     } else if (run_step == 0) {
    //         MotorControl_SetMode(&motor, POSITION_MODE);
    //         MotorControl_SetTarget(&motor, 8000);
    //         MODBUS_SyncTargetPosition(8000);      // 同步pending，上位机可见
    //     }
    // }
	  /* ========== 潜水模拟时间轴（PWM开环模式）==========
	   * 总时长 2分30秒, 分5个阶段
	   *-------------------------------------------------------------------
	   * 阶段   时间          振动PWM              皮带张力
	   *-------------------------------------------------------------------
	   * 浅海   0:00-0:30     60 恒定              LEVEL_0(0) 不收紧
	   * 中层   0:30-1:10     85 恒定              0→LEVEL_MID(350)
	   * 深海   1:10-1:50     80↔130 脉冲          MID→LEVEL_DEEP(750)
	   *                     (每800ms切换, 周期1.6s)
	   * 上浮   1:50-2:30     100→20 线性递减       DEEP→0 释放
	   *                     (vib = 100 - 80*p)
	   * 结束   2:30+         0                    running=false 全部停止
	   *-------------------------------------------------------------------
	   * 注: 皮带张力为位置控制(需切POSITION_MODE), 此处仅实现振动PWM开环部分;
	   *     皮带目标位置以注释标注, 如需启用请配合第二电机或模式切换实现
	   */

    /* ============ 系统任务（勿删）============ */

    /* 处理PB4/PB5输入引脚 */
    MotorControl_ProcessInputs();

    /* 处理MODBUS�????? */
    MODBUS_Process(&mb_ctx);

    /* �?????测UART接收是否意外停止（ORE等错误后HAL会中止），自动重�????? */
    if (huart1.RxState == HAL_UART_STATE_READY) {
      HAL_UART_Receive_IT(&huart1, &mb_ctx.rx_byte, 1);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* STM32F1的ADC必须执行校准, 否则HAL_ADC_Start_DMA会卡死或转换结果异常
   * 校准必须在ADC关闭状态下执行, HAL_ADCEx_Calibration_Start内部会先关ADC再校准 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin DE_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  UART接收完成回调（每收到1字节触发�?????
  *         将字节存入MODBUS接收缓冲区，并重新启动下�?????字节接收
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* 将收到的字节交给MODBUS处理 */
        MODBUS_OnByteReceived(&mb_ctx, mb_ctx.rx_byte);
        /* 重新启动下一字节的中断接收.
         * 注意: HAL_UART_Receive_IT在gState=BUSY_TX(发送中)时会返回HAL_BUSY静默失败,
         *   接收会被永久关闭, 后续字节全部丢失. 必须检查返回值, 失败时直接操作寄存器. */
        if (HAL_UART_Receive_IT(&huart1, &mb_ctx.rx_byte, 1) != HAL_OK) {
            /* 发送进行中导致HAL拒绝重启接收: 直接使能RXNEIE保证后续字节能触发中断.
             * 发送完成后, HAL_UART_IRQHandler的TX路径不会关闭RXNEIE,
             * 接收中断会继续触发, 调用本回调重启HAL接收流程. */
            SET_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE);
        }
    }
}

/**
  * @brief  UART错误回调 - ORE/FE/NE错误会中止接收，必须重启
  * 注意: HAL_UART_Receive_IT在huart->gState=BUSY_TX时会返回HAL_BUSY静默失败,
  *       导致接收永久卡死. 必须强制把gState切回READY再重启接收.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* 清除溢出错误标志 (ORE: 软件读SR然后读DR清零, 或直接清ORECF) */
        __HAL_UART_CLEAR_OREFLAG(huart);
        /* 清除帧错误/噪声错误标志 */
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);

        /* 强制把HAL状态切回READY.
         * 错误发生时HAL可能把RxState置为READY但gState仍为BUSY_TX(发送中),
         * 直接调HAL_UART_Receive_IT会因gState!=READY返回HAL_BUSY, 接收永远无法重启.
         * 此时必须改用直接寄存器操作重新使能RXNE中断, 绕过HAL状态机. */
        if (huart->gState != HAL_UART_STATE_READY) {
            /* 发送仍在进行: 不能调HAL_UART_Receive_IT(会返回BUSY),
             * 直接使能RXNEIE, 硬件收到下一字节会立即触发中断.
             * HAL_UART_IRQHandler会处理RXNE, 即使gState=BUSY_TX也能读DR. */
            SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
        } else {
            /* 发送已完成: 正常重启接收 */
            HAL_UART_Receive_IT(&huart1, &mb_ctx.rx_byte, 1);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
