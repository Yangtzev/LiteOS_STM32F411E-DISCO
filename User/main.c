/**
  ******************************************************************************
  * @file    main.c
  * @author  Sakura_L
  * @version V1.0
  * @date    2026-04-25
  * @brief   LiteOS task for default, tilt and activity-detection LED modes
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include <math.h>
#include <string.h>

/* LiteOS headers */
#include "los_sys.h"
#include "los_task.ph"

/* Board headers */
#include "./usart/bsp_debug_usart.h"
#include "stm32f411e_discovery.h"
#include "stm32f411e_discovery_accelerometer.h"
#include "app_x-cube-ai.h"
#include "app_config.h"

#include "app_hw.h"
#include "app_button.h"
#include "app_modes.h"
#include "app_activity.h"

/* Private function prototypes -----------------------------------------------*/
static UINT32 AppTaskCreate(void);
static UINT32 CreateLedControlTask(void);
static void LedControlTask(void);
static uint8_t IsModeAvailable(LedMode_TypeDef mode, uint8_t acceleroReady, uint8_t aiReady);
static LedMode_TypeDef GetNextMode(LedMode_TypeDef currentMode, uint8_t acceleroReady, uint8_t aiReady);

extern void BSP_Init(void);


/* 程序入口：初始化 HAL、板级支持（BSP）、内核，创建应用任务并启动 LiteOS 调度。 */

int main(void)
{
  UINT32 uwRet = LOS_OK;

  HAL_Init();
  BSP_Init();

  printf("STM32-LiteOS\r\n");
  printf("KEY1 cycles default mode, accelerometer mode and activity mode.\r\n");

  uwRet = LOS_KernelInit();							//LOS_KernelInit函数：return LOS_OK
  if (uwRet != LOS_OK)
  {
    printf("LiteOS error 0x%X\r\n", uwRet);
    return LOS_NOK;
  }

  uwRet = AppTaskCreate();							// uwRet = LOS_TaskCreate(&LedControlTaskHandle, &task_init_param)
  if (uwRet != LOS_OK)
  {
    printf("AppTaskCreate error 0x%X\r\n", uwRet);
    return LOS_NOK;
  }

  LOS_Start();										//启动，进入任务

  while (1)
  {
  }
}

/* 创建应用任务：在此示例中用于创建 LED 控制任务。 */

static UINT32 AppTaskCreate(void)
{
  return CreateLedControlTask();
}
 
static UINT32 CreateLedControlTask(void)
{
  UINT32 uwRet = LOS_OK;
  TSK_INIT_PARAM_S task_init_param = {0};

  task_init_param.usTaskPrio = 5;
  task_init_param.pcName = "LedControlTask";
  task_init_param.pfnTaskEntry = (TSK_ENTRY_FUNC)LedControlTask;
  task_init_param.uwStackSize = 4096;

  uwRet = LOS_TaskCreate(&LedControlTaskHandle, &task_init_param);
  return uwRet;
}

/* LED 控制任务主循环：配置 PWM、初始化传感器/AI，轮询按键并根据当前模式更新 LED。 */
static void LedControlTask(void)
{
  uint8_t acceleroReady = 0U;
  uint8_t aiReady = 0U;

  TIM4_Config();
  ApplyDefaultMode();

  if (BSP_ACCELERO_Init() == ACCELERO_OK)
  {
    acceleroReady = 1U;
  }
  else
  {
    printf("BSP_ACCELERO_Init failed, keep default mode only.\r\n");
  }

  if (acceleroReady != 0U)
  {
    if (aiInit() == 0)
    {
      aiReady = 1U;
    }
    else
    {
      printf("AI init failed, activity mode disabled.\r\n");
    }
  }

  while (1)
  {
    if (PollButtonPressedEvent() != 0U)
    {
      CurrentMode = GetNextMode(CurrentMode, acceleroReady, aiReady);

      if (CurrentMode == LED_MODE_DEFAULT)
      {
        ApplyDefaultMode();
        printf("Switch to default mode.\r\n");
      }
      else if (CurrentMode == LED_MODE_TILT)
      {
        printf("Switch to accelerometer mode.\r\n");
      }
      else
      {
        ResetActivityWindow();
        ClearAllLeds();
        printf("Switch to activity mode.\r\n");
      }
    }

    if (CurrentMode == LED_MODE_TILT)
    {
      ApplyTiltMode();
    }
    else if (CurrentMode == LED_MODE_ACTIVITY)
    {
      ApplyActivityMode();
    }

    LOS_TaskDelay(BUTTON_SCAN_MS);
  }
}

/* 判断目标模式是否可用：默认总可用；倾斜模式需加速度计；活动模式需加速度计与 AI。 */

static uint8_t IsModeAvailable(LedMode_TypeDef mode, uint8_t acceleroReady, uint8_t aiReady)
{
  switch (mode)
  {
    case LED_MODE_DEFAULT:
      return 1U;

    case LED_MODE_TILT:
      return acceleroReady;

    case LED_MODE_ACTIVITY:
      return (uint8_t)((acceleroReady != 0U) && (aiReady != 0U));

    default:
      return 0U;
  }
}


/* 在模式循环中查找下一个可用模式，遇到不可用则继续尝试下一个。 */

static LedMode_TypeDef GetNextMode(LedMode_TypeDef currentMode, uint8_t acceleroReady, uint8_t aiReady)
{
  uint32_t attempt;
  LedMode_TypeDef nextMode = currentMode;

  for (attempt = 0U; attempt < (uint32_t)LED_MODE_COUNT; ++attempt)
  {
    nextMode = (LedMode_TypeDef)(((uint32_t)nextMode + 1U) % (uint32_t)LED_MODE_COUNT);
    if (IsModeAvailable(nextMode, acceleroReady, aiReady) != 0U)
    {
      return nextMode;
    }
  }

  return LED_MODE_DEFAULT;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
