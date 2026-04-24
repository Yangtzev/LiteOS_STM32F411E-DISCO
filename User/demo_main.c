/* 包含头文件 ------------------------------------------------------------------*/
#include "main.h"

/* 私有类型定义 -----------------------------------------------------------*/
/* 私有定义 ------------------------------------------------------------*/
#define KEY_PRESSED     0x01
#define KEY_NOT_PRESSED 0x00

/* 按键防抖时间（毫秒） */
#define DEBOUNCE_MS     50U

/* TIM4 自动重装载与比较寄存器初始值 */
/* 使用较小的 ARR 以获得更高的 PWM 频率（用于亮度控制） */
#define TIM_ARR        (uint16_t)255
#define TIM_CCR        (uint16_t)128
/* 期望的 PWM 频率（Hz） */
#define PWM_FREQ       1000U
/* 用于缩放的期望最大传感器读数（如有需要请调整） */
#define MAX_SENSOR     3000U
#define CURSOR_STEP     7

/* 私有宏 -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x
#define MAX_AB(a,b)       (a < b) ? (b) : a

/* 私有变量 ---------------------------------------------------------*/
__IO uint8_t UserButtonPressed = 0x00;
__IO uint8_t DemoEnterCondition = 0x00;
/* 上次接受按键事件的滴答（用于防抖） */
__IO uint32_t LastButtonTick = 0;

/* 加速度计使用的变量 */
__IO int16_t X_Offset, Y_Offset;
int16_t Buffer[3];

/* MEMS 阈值 {低/高} */
static int16_t ThreadholdAcceleroLow = -1500, ThreadholdAcceleroHigh = 1500;

/* 定时器使用的变量 */
uint16_t PrescalerValue = 0;
TIM_HandleTypeDef htim4;
TIM_OC_InitTypeDef sConfigTim4;

/* SysTick ISR 中使用的变量 */
uint8_t Counter  = 0x00;
__IO uint16_t MaxAcceleration = 0;

/* 私有函数 -----------------------------------------------*/
static void TIM4_Config(void);
static void Demo_Exec(void);
static void SystemClock_Config(void);


int main(void)
{
  /* STM32F4xx HAL 库初始化：
       - 配置 Flash 预取、指令和数据缓存
       - 配置 SysTick 以产生每 1 毫秒一次的中断
       - 将 NVIC 组优先级设置为 4
       - 全局 MSP（MCU 支持包）初始化
     */
  HAL_Init();

  /* 配置 LED4、LED3、LED5 和 LED6 */
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);

  /* 配置系统时钟为 84 MHz */
  SystemClock_Config();

  /* 初始化用户按键 */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* 执行演示程序 */
  Demo_Exec();

  while (1)
  {
  }
}

static void Demo_Exec(void)
{
  /* 初始化加速度传感器（MEMS）*/
  if(BSP_ACCELERO_Init() != HAL_OK)
  {
  }

  while(1)
  {
    DemoEnterCondition = 0x00;

    /* 重置用户按键标志 */
	UserButtonPressed = 0x00;

    /* 将 LED 配置为通过 GPIO 控制 */
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    BSP_LED_Init(LED5);
    BSP_LED_Init(LED6);

    /* SysTick 每 10ms 触发一次 */
    SystemCoreClock = HAL_RCC_GetHCLKFreq();
    SysTick_Config(SystemCoreClock / 100);

    /* 关闭所有 LED */
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED5);
    BSP_LED_Off(LED6);

    /* 等待用户按键按下 */
    while (UserButtonPressed == 0x00)
    {      
      BSP_LED_On(LED4);
      BSP_LED_On(LED3);
      BSP_LED_On(LED5);
      BSP_LED_On(LED6);
    }

    /* 等待用户按键释放 */
    while (BSP_PB_GetState(BUTTON_KEY) != KEY_NOT_PRESSED)
    {}
    UserButtonPressed = 0x00;
    /* TIM4 通道配置 */
    TIM4_Config();

    DemoEnterCondition = 0x01;

    /* 等待用户按键按下 */
    while (UserButtonPressed == 0x00)
    {}

    /* 等待用户按键释放 */
    while (BSP_PB_GetState(BUTTON_KEY) != KEY_NOT_PRESSED)
    {}

    if(HAL_TIM_PWM_DeInit(&htim4) != HAL_OK)
    {
    }
  }
}

/**
  * @brief  配置 TIM 外设
  * @param  无
  * @retval 无
  */
static void TIM4_Config(void)
{
  /* -----------------------------------------------------------------------
  TIM4 配置：输出比较（PWM）模式：
  在此示例中，TIM4 的输入时钟（TIM4CLK）设置为 2 * APB1 时钟（PCLK1），
  因为 APB1 的预分频不为 1（APB1 Prescaler = 4，详见 system_stm32f4xx.c）。
  TIM4CLK = 2 * PCLK1
  PCLK1 = HCLK / 4
  => TIM4CLK = 2*(HCLK / 4) = HCLK/2 = SystemCoreClock/2
  例如要得到 TIM4 计数器时钟为 2 KHz，预分频器计算如下：
  预分频 = (TIM4CLK / TIM4 计数器时钟) - 1
  若要得到 TIM4 输出为 1 Hz，周期（ARR）按如下公式计算：
  ARR = (TIM4 counter clock / TIM4 output clock) - 1
  TIM4 各通道占空比计算示例：
  TIM4 Channelx 占空比 = (TIM4_CCRx / TIM4_ARR) * 100
  ----------------------------------------------------------------------- */
  /* 为所需 PWM 频率计算预分频值 */
  /* 预分频计算公式：Prescaler = (TIM4CLK / (PWM_FREQ * (ARR+1))) - 1 */
  PrescalerValue = (uint16_t)(((SystemCoreClock / 2) / (PWM_FREQ * (TIM_ARR + 1U))) - 1U);

  /* 时间基（计数器）配置 */
  htim4.Instance             = TIM4;
  htim4.Init.Period          = TIM_ARR;
  htim4.Init.Prescaler       = PrescalerValue;
  htim4.Init.ClockDivision   = 0;
  htim4.Init.CounterMode     = TIM_COUNTERMODE_UP;
  if(HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {  }

  /* TIM PWM1 模式配置：通道 */
  /* 输出比较（PWM）模式配置：通道1 */
  sConfigTim4.OCMode = TIM_OCMODE_PWM1;
  sConfigTim4.OCIdleState = TIM_CCx_ENABLE;
  /* 初始占空比为 0（LED 关闭） */
  sConfigTim4.Pulse = 0;
  sConfigTim4.OCPolarity = TIM_OCPOLARITY_HIGH;

  /* 输出比较（PWM）模式配置：通道1 */
  if(HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigTim4, TIM_CHANNEL_1) != HAL_OK)
  {  }
  /* 输出比较（PWM）模式配置：通道2 */
  if(HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigTim4, TIM_CHANNEL_2) != HAL_OK)
  {  }
  /* 输出比较（PWM）模式配置：通道3 */
  if(HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigTim4, TIM_CHANNEL_3) != HAL_OK)
  {  }
  /* 输出比较（PWM）模式配置：通道4 */
  if(HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigTim4, TIM_CHANNEL_4) != HAL_OK)
  {  }

  /* 确保比较寄存器初始化为 0 并启动所有 PWM 通道，
     这样运行时只需修改 CCR，而不用频繁 Stop/Start，避免可见闪烁 */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/**
  * @brief  SysTick 回调函数
  * @param  无
  * @retval 无
  */
void HAL_SYSTICK_Callback(void)
{
    Counter ++;
    if (Counter == 10)
    {
      /* 重置用于读取加速度的缓冲 */
      Buffer[0] = 0;
      Buffer[1] = 0;

      /* 读取加速度 */
      BSP_ACCELERO_GetXYZ(Buffer);

      /* 设置 X 与 Y 位置 */
      X_Offset = Buffer[0];
      Y_Offset = Buffer[1];

      /* 将超过阈值的幅度映射到比较寄存器 CCR（范围 0..TIM_ARR） */
      uint32_t magX = (uint32_t)ABS(X_Offset);
      uint32_t magY = (uint32_t)ABS(Y_Offset);
      uint32_t brightX = 0;
      uint32_t brightY = 0;

      if (magX > (uint32_t)ThreadholdAcceleroHigh)
      {
        uint32_t v = magX - (uint32_t)ThreadholdAcceleroHigh;
        brightX = (v * (uint32_t)TIM_ARR) / (MAX_SENSOR - (uint32_t)ThreadholdAcceleroHigh);
        if (brightX > TIM_ARR) brightX = TIM_ARR;
      }

      if (magY > (uint32_t)ThreadholdAcceleroHigh)
      {
        uint32_t v = magY - (uint32_t)ThreadholdAcceleroHigh;
        brightY = (v * (uint32_t)TIM_ARR) / (MAX_SENSOR - (uint32_t)ThreadholdAcceleroHigh);
        if (brightY > TIM_ARR) brightY = TIM_ARR;
      }

      /* 左方向 -> TIM_CHANNEL_1 */
      if (X_Offset < ThreadholdAcceleroLow)
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, brightX);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
      }

      /* 右方向 -> TIM_CHANNEL_3 */
      if (X_Offset > ThreadholdAcceleroHigh)
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, brightX);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      }

      /* 上方向 -> TIM_CHANNEL_4 */
      if (Y_Offset > ThreadholdAcceleroHigh)
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, brightY);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      }

      /* 下方向 -> TIM_CHANNEL_2 */
      if (Y_Offset < ThreadholdAcceleroLow)
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, brightY);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
      }

      Counter = 0x00;
    }
}

/**
  * @brief  EXTI 中断回调
  * @param  GPIO_Pin: 指定与 EXTI 线相连的引脚
  * @retval 无
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_BUTTON_PIN)
  {
    uint32_t now = HAL_GetTick();
    /* 仅在距离上次接受事件超过 DEBOUNCE_MS 时才接受本次事件 */
    if ((now - LastButtonTick) > DEBOUNCE_MS)
    {
      UserButtonPressed = 0x01;
      LastButtonTick = now;
    }
  }
}


static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

    /* 使能电源控制时钟 */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* 电压缩放用于在设备低于最大频率运行时优化功耗；
      关于不同系统频率下的电压缩放设置，请参阅器件数据手册。 */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* 使能 HSE 振荡器并以 HSE 为 PLL 源 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /* 选择 PLL 作为系统时钟源，并配置 HCLK、PCLK1 和 PCLK2 的分频 */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
