/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2017-xx-xx
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

/* Private defines -----------------------------------------------------------*/
#define LED_PWM_PERIOD          ((uint16_t)255U)
#define LED_PWM_FREQ_HZ         (1000U)

#define BUTTON_SCAN_MS          (20U)
#define BUTTON_DEBOUNCE_COUNT   (3U)
#define BUTTON_PRESSED          GPIO_PIN_SET
#define BUTTON_RELEASED         GPIO_PIN_RESET

#define ACCEL_THRESHOLD_LOW_MG  (-1500)
#define ACCEL_THRESHOLD_HIGH_MG (1500)
#define ACCEL_MAX_SENSOR_MG     (3000U)
#define ACCEL_MG_TO_MS2         (0.00980665f)

#define ACTIVITY_WINDOW_LEN     ((uint32_t)24U)
#define ACTIVITY_AXIS_NUM       ((uint32_t)3U)
#define ACTIVITY_SAMPLE_MS      ((uint32_t)40U)
#define ACTIVITY_SAMPLE_LOOPS   ((ACTIVITY_SAMPLE_MS + BUTTON_SCAN_MS - 1U) / BUTTON_SCAN_MS)
#define AI_FILTER_ORDER         ((uint32_t)4U)
#define AI_EPSILON              (1.0e-6f)

#define ACTIVITY_LABEL_STATIONARY  "Stationary"
#define ACTIVITY_LABEL_WALKING     "Walking"
#define ACTIVITY_LABEL_JOGGING     "Jogging"
#define ACTIVITY_LABEL_BIKING      "Biking"

/* Private types -------------------------------------------------------------*/
typedef enum
{
  LED_MODE_DEFAULT = 0,
  LED_MODE_TILT,
  LED_MODE_ACTIVITY,
  LED_MODE_COUNT
} LedMode_TypeDef;

typedef enum
{
  ACTIVITY_STATIONARY = 0,
  ACTIVITY_WALKING,
  ACTIVITY_JOGGING,
  ACTIVITY_BIKING,
  ACTIVITY_UNKNOWN
} ActivityClass_TypeDef;

/* Private variables ---------------------------------------------------------*/
static UINT32 LedControlTaskHandle;
static TIM_HandleTypeDef Tim4Handle;
static LedMode_TypeDef CurrentMode = LED_MODE_DEFAULT;
static int16_t AcceleroBuffer[3];
static float ActivityWindow[ACTIVITY_WINDOW_LEN][ACTIVITY_AXIS_NUM];
static float ActivityOutput[STAI_NETWORK_OUT_1_SIZE];
static uint32_t ActivitySampleCount = 0U;
static uint32_t ActivitySampleLoopCount = 0U;
static ActivityClass_TypeDef LastActivityClass = ACTIVITY_UNKNOWN;

static const float ActivityFilterACoeff[AI_FILTER_ORDER + 1U] = {
  1.0f, -3.868656635f, 5.614526749f, -3.622760773f, 0.8768966198f
};
static const float ActivityFilterBCoeff[AI_FILTER_ORDER + 1U] = {
  0.9364275932f, -3.745710373f, 5.618565559f, -3.745710373f, 0.9364275932f
};
static const float ActivityFilterInitialState[AI_FILTER_ORDER] = {
  -0.936528250873f, 2.809571532101f, -2.809559172096f, 0.936515859573f
};

/* Private function prototypes -----------------------------------------------*/
static UINT32 AppTaskCreate(void);
static UINT32 CreateLedControlTask(void);
static void LedControlTask(void);
static void TIM4_Config(void);
static uint32_t GetTim4Clock(void);
static uint8_t PollButtonPressedEvent(void);
static uint32_t ScaleTiltToDuty(int32_t axisMg);
static void SetLedDuty(uint32_t channel, uint32_t duty);
static void SetActivityLed(ActivityClass_TypeDef activity);
static void ClearAllLeds(void);
static void ApplyDefaultMode(void);
static void ApplyTiltMode(void);
static uint8_t IsModeAvailable(LedMode_TypeDef mode, uint8_t acceleroReady, uint8_t aiReady);
static LedMode_TypeDef GetNextMode(LedMode_TypeDef currentMode, uint8_t acceleroReady, uint8_t aiReady);
static void ResetActivityWindow(void);
static void HighPassFilterAxis(const float *input, float *output, uint32_t length);
static void PreprocessActivityWindow(float input[][ACTIVITY_AXIS_NUM],
                                    float output[][ACTIVITY_AXIS_NUM]);
static ActivityClass_TypeDef GetBestActivityClass(const float *scores, uint32_t count);
static const char *GetActivityName(ActivityClass_TypeDef activity);
static void ApplyActivityMode(void);


extern void BSP_Init(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  UINT32 uwRet = LOS_OK;

  HAL_Init();
  BSP_Init();

  printf("STM32-LiteOS\r\n");
  printf("KEY1 cycles default mode, accelerometer mode and activity mode.\r\n");

  uwRet = LOS_KernelInit();
  if (uwRet != LOS_OK)
  {
    printf("LiteOS error 0x%X\r\n", uwRet);
    return LOS_NOK;
  }

  uwRet = AppTaskCreate();
  if (uwRet != LOS_OK)
  {
    printf("AppTaskCreate error 0x%X\r\n", uwRet);
    return LOS_NOK;
  }

  LOS_Start();

  while (1)
  {
  }
}

/**
  * @brief  Create application tasks
  * @param  None
  * @retval LOS status
  */
static UINT32 AppTaskCreate(void)
{
  return CreateLedControlTask();
}

/**
  * @brief  Create LED control task
  * @param  None
  * @retval LOS status
  */
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

/**
  * @brief  LED control task
  * @param  None
  * @retval None
  */
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

/**
  * @brief  Configure TIM4 PWM on the four LED pins
  * @param  None
  * @retval None
  */
static void TIM4_Config(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  uint32_t timerClock = GetTim4Clock();
  uint32_t prescaler = (timerClock / (LED_PWM_FREQ_HZ * ((uint32_t)LED_PWM_PERIOD + 1U))) - 1U;

  Tim4Handle.Instance = TIM4;
  Tim4Handle.Init.Period = LED_PWM_PERIOD;
  Tim4Handle.Init.Prescaler = (uint16_t)prescaler;
  Tim4Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  Tim4Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_PWM_Init(&Tim4Handle) != HAL_OK)
  {
    while (1)
    {
    }
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

  if (HAL_TIM_PWM_ConfigChannel(&Tim4Handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    while (1)
    {
    }
  }
  if (HAL_TIM_PWM_ConfigChannel(&Tim4Handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    while (1)
    {
    }
  }
  if (HAL_TIM_PWM_ConfigChannel(&Tim4Handle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    while (1)
    {
    }
  }
  if (HAL_TIM_PWM_ConfigChannel(&Tim4Handle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    while (1)
    {
    }
  }

  if (HAL_TIM_PWM_Start(&Tim4Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    while (1)
    {
    }
  }
  if (HAL_TIM_PWM_Start(&Tim4Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    while (1)
    {
    }
  }
  if (HAL_TIM_PWM_Start(&Tim4Handle, TIM_CHANNEL_3) != HAL_OK)
  {
    while (1)
    {
    }
  }
  if (HAL_TIM_PWM_Start(&Tim4Handle, TIM_CHANNEL_4) != HAL_OK)
  {
    while (1)
    {
    }
  }
}

/**
  * @brief  Return the TIM4 input clock
  * @param  None
  * @retval Timer clock in Hz
  */
static uint32_t GetTim4Clock(void)
{
  RCC_ClkInitTypeDef clkInitStruct = {0};
  uint32_t flashLatency = 0U;
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();

  HAL_RCC_GetClockConfig(&clkInitStruct, &flashLatency);

  if (clkInitStruct.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    return pclk1;
  }

  return (pclk1 * 2U);
}

/**
  * @brief  Debounced button press detection
  * @param  None
  * @retval 1 when a stable press is detected, otherwise 0
  */
static uint8_t PollButtonPressedEvent(void)
{
  static uint32_t lastSample = BUTTON_RELEASED;
  static uint32_t stableState = BUTTON_RELEASED;
  static uint32_t stableCount = 0U;
  uint32_t currentSample = BSP_PB_GetState(BUTTON_KEY);

  if (currentSample == lastSample)
  {
    if (stableCount < BUTTON_DEBOUNCE_COUNT)
    {
      stableCount++;
    }
  }
  else
  {
    lastSample = currentSample;
    stableCount = 1U;
  }

  if ((stableCount >= BUTTON_DEBOUNCE_COUNT) && (stableState != currentSample))
  {
    stableState = currentSample;
    if (stableState == BUTTON_PRESSED)
    {
      return 1U;
    }
  }

  return 0U;
}

/**
  * @brief  Map tilt magnitude to LED duty cycle
  * @param  axisMg Absolute acceleration on one axis in mg
  * @retval PWM duty value
  */
static uint32_t ScaleTiltToDuty(int32_t axisMg)
{
  uint32_t magnitude = (uint32_t)axisMg;
  uint32_t threshold = (uint32_t)ACCEL_THRESHOLD_HIGH_MG;
  uint32_t duty;

  if (magnitude <= threshold)
  {
    return 0U;
  }

  duty = (uint32_t)(((magnitude - threshold) * LED_PWM_PERIOD) /
                    (ACCEL_MAX_SENSOR_MG - threshold));
  if (duty > LED_PWM_PERIOD)
  {
    duty = LED_PWM_PERIOD;
  }

  return duty;
}

/**
  * @brief  Update one PWM channel
  * @param  channel TIM4 channel
  * @param  duty PWM duty
  * @retval None
  */
static void SetLedDuty(uint32_t channel, uint32_t duty)
{
  if (duty > LED_PWM_PERIOD)
  {
    duty = LED_PWM_PERIOD;
  }

  __HAL_TIM_SET_COMPARE(&Tim4Handle, channel, duty);
}

/**
  * @brief  Turn off all LEDs
  * @param  None
  * @retval None
  */
static void ClearAllLeds(void)
{
  SetLedDuty(TIM_CHANNEL_1, 0U);
  SetLedDuty(TIM_CHANNEL_2, 0U);
  SetLedDuty(TIM_CHANNEL_3, 0U);
  SetLedDuty(TIM_CHANNEL_4, 0U);
}

/**
  * @brief  Default mode: all four LEDs on
  * @param  None
  * @retval None
  */
static void ApplyDefaultMode(void)
{
  SetLedDuty(TIM_CHANNEL_1, LED_PWM_PERIOD);
  SetLedDuty(TIM_CHANNEL_2, LED_PWM_PERIOD);
  SetLedDuty(TIM_CHANNEL_3, LED_PWM_PERIOD);
  SetLedDuty(TIM_CHANNEL_4, LED_PWM_PERIOD);
}

/**
  * @brief  Tilt mode: light LEDs according to board direction
  * @param  None
  * @retval None
  */
static void ApplyTiltMode(void)
{
  int16_t x;
  int16_t y;
  uint32_t leftDuty = 0U;
  uint32_t rightDuty = 0U;
  uint32_t upDuty = 0U;
  uint32_t downDuty = 0U;
  
  BSP_ACCELERO_GetXYZ(AcceleroBuffer);

  x = AcceleroBuffer[0];
  y = AcceleroBuffer[1];

  if (x < ACCEL_THRESHOLD_LOW_MG)
  {
    leftDuty = ScaleTiltToDuty(-(int32_t)x);
  }
  else if (x > ACCEL_THRESHOLD_HIGH_MG)
  {
    rightDuty = ScaleTiltToDuty((int32_t)x);
  }

  if (y < ACCEL_THRESHOLD_LOW_MG)
  {
    downDuty = ScaleTiltToDuty(-(int32_t)y);
  }
  else if (y > ACCEL_THRESHOLD_HIGH_MG)
  {
    upDuty = ScaleTiltToDuty((int32_t)y);
  }

  /* TIM4 CH1/CH2/CH3/CH4 -> LED4(left), LED3(down), LED5(right), LED6(up) */
  SetLedDuty(TIM_CHANNEL_1, leftDuty);
  SetLedDuty(TIM_CHANNEL_2, downDuty);
  SetLedDuty(TIM_CHANNEL_3, rightDuty);
  SetLedDuty(TIM_CHANNEL_4, upDuty);
}

/**
  * @brief  Set LED according to the detected activity
  * @param  activity Detected activity class
  * @retval None
  */
static void SetActivityLed(ActivityClass_TypeDef activity)
{
  ClearAllLeds();

  switch (activity)
  {
    case ACTIVITY_BIKING:
      SetLedDuty(TIM_CHANNEL_1, LED_PWM_PERIOD);
      break;

    case ACTIVITY_WALKING:
      SetLedDuty(TIM_CHANNEL_2, LED_PWM_PERIOD);
      break;

    case ACTIVITY_JOGGING:
      SetLedDuty(TIM_CHANNEL_3, LED_PWM_PERIOD);
      break;

    case ACTIVITY_STATIONARY:
      SetLedDuty(TIM_CHANNEL_4, LED_PWM_PERIOD);
      break;

    default:
      break;
  }
}

/**
  * @brief  Check whether a mode can be entered
  * @param  mode Target mode
  * @param  acceleroReady Accelerometer availability
  * @param  aiReady AI availability
  * @retval 1 when available, otherwise 0
  */
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

/**
  * @brief  Get next available mode in the cycle
  * @param  currentMode Current mode
  * @param  acceleroReady Accelerometer availability
  * @param  aiReady AI availability
  * @retval Next available mode
  */
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

/**
  * @brief  Reset activity window collection state
  * @param  None
  * @retval None
  */
static void ResetActivityWindow(void)
{
  memset(ActivityWindow, 0, sizeof(ActivityWindow));
  memset(ActivityOutput, 0, sizeof(ActivityOutput));
  ActivitySampleCount = 0U;
  ActivitySampleLoopCount = 0U;
  LastActivityClass = ACTIVITY_UNKNOWN;
}

/**
  * @brief  High-pass filter one sensor axis
  * @param  input Input samples
  * @param  output Filtered samples
  * @param  length Number of samples
  * @retval None
  */
static void HighPassFilterAxis(const float *input, float *output, uint32_t length)
{
  float state[AI_FILTER_ORDER];
  uint32_t i;

  if ((input == NULL) || (output == NULL) || (length == 0U))
  {
    return;
  }

  if (fabsf(input[0]) <= AI_EPSILON)
  {
    for (i = 0U; i < AI_FILTER_ORDER; ++i)
    {
      state[i] = ActivityFilterInitialState[i];
    }
  }
  else
  {
    for (i = 0U; i < AI_FILTER_ORDER; ++i)
    {
      state[i] = ActivityFilterInitialState[i] * input[0];
    }
  }

  for (i = 0U; i < length; ++i)
  {
    float y = (ActivityFilterBCoeff[0] * input[i]) + state[0];

    state[0] = (ActivityFilterBCoeff[1] * input[i]) + state[1] - (ActivityFilterACoeff[1] * y);
    state[1] = (ActivityFilterBCoeff[2] * input[i]) + state[2] - (ActivityFilterACoeff[2] * y);
    state[2] = (ActivityFilterBCoeff[3] * input[i]) + state[3] - (ActivityFilterACoeff[3] * y);
    state[3] = (ActivityFilterBCoeff[4] * input[i]) - (ActivityFilterACoeff[4] * y);

    output[i] = y;
  }
}

/**
  * @brief  Apply gravity rotation and suppression to one window
  * @param  input Raw window samples
  * @param  output Preprocessed window samples
  * @retval None
  */
static void PreprocessActivityWindow(float input[][ACTIVITY_AXIS_NUM],
                                    float output[][ACTIVITY_AXIS_NUM])
{
  float axisInput[ACTIVITY_AXIS_NUM][ACTIVITY_WINDOW_LEN];
  float axisDyn[ACTIVITY_AXIS_NUM][ACTIVITY_WINDOW_LEN];
  uint32_t sample;
  uint32_t axis;

  for (sample = 0U; sample < ACTIVITY_WINDOW_LEN; ++sample)
  {
    for (axis = 0U; axis < ACTIVITY_AXIS_NUM; ++axis)
    {
      axisInput[axis][sample] = input[sample][axis];
    }
  }

  for (axis = 0U; axis < ACTIVITY_AXIS_NUM; ++axis)
  {
    HighPassFilterAxis(axisInput[axis], axisDyn[axis], ACTIVITY_WINDOW_LEN);
  }

  for (sample = 0U; sample < ACTIVITY_WINDOW_LEN; ++sample)
  {
    float gravity[ACTIVITY_AXIS_NUM];
    float gravityNorm;
    float rotAxis[ACTIVITY_AXIS_NUM];
    float rotAxisNorm;
    float sinAngle;
    float cosAngle;
    float dyn[ACTIVITY_AXIS_NUM];
    float cross[ACTIVITY_AXIS_NUM];
    float axisDotDyn;

    for (axis = 0U; axis < ACTIVITY_AXIS_NUM; ++axis)
    {
      dyn[axis] = axisDyn[axis][sample];
      gravity[axis] = axisInput[axis][sample] - dyn[axis];
    }

    gravityNorm = sqrtf((gravity[0] * gravity[0]) +
                        (gravity[1] * gravity[1]) +
                        (gravity[2] * gravity[2]));

    if (gravityNorm > AI_EPSILON)
    {
      gravity[0] /= gravityNorm;
      gravity[1] /= gravityNorm;
      gravity[2] /= gravityNorm;
    }
    else
    {
      gravity[0] = 0.0f;
      gravity[1] = 0.0f;
      gravity[2] = -1.0f;
    }

    rotAxis[0] = -gravity[1];
    rotAxis[1] = gravity[0];
    rotAxis[2] = 0.0f;

    rotAxisNorm = sqrtf((rotAxis[0] * rotAxis[0]) + (rotAxis[1] * rotAxis[1]));
    sinAngle = rotAxisNorm;
    cosAngle = -gravity[2];

    if (rotAxisNorm > AI_EPSILON)
    {
      rotAxis[0] /= rotAxisNorm;
      rotAxis[1] /= rotAxisNorm;
    }
    else
    {
      rotAxis[0] = 0.0f;
      rotAxis[1] = 0.0f;
      rotAxis[2] = 0.0f;
      sinAngle = 0.0f;
    }

    cross[0] = (rotAxis[1] * dyn[2]) - (rotAxis[2] * dyn[1]);
    cross[1] = (rotAxis[2] * dyn[0]) - (rotAxis[0] * dyn[2]);
    cross[2] = (rotAxis[0] * dyn[1]) - (rotAxis[1] * dyn[0]);

    axisDotDyn = (rotAxis[0] * dyn[0]) + (rotAxis[1] * dyn[1]) + (rotAxis[2] * dyn[2]);

    output[sample][0] = (dyn[0] * cosAngle) + (cross[0] * sinAngle) +
                        (rotAxis[0] * axisDotDyn * (1.0f - cosAngle));
    output[sample][1] = (dyn[1] * cosAngle) + (cross[1] * sinAngle) +
                        (rotAxis[1] * axisDotDyn * (1.0f - cosAngle));
    output[sample][2] = (dyn[2] * cosAngle) + (cross[2] * sinAngle) +
                        (rotAxis[2] * axisDotDyn * (1.0f - cosAngle));
  }
}

/**
  * @brief  Get the argmax class from inference scores
  * @param  scores Output scores
  * @param  count Number of classes
  * @retval Best class
  */
static ActivityClass_TypeDef GetBestActivityClass(const float *scores, uint32_t count)
{
  uint32_t i;
  uint32_t bestIndex = 0U;
  float bestScore;

  if ((scores == NULL) || (count == 0U))
  {
    return ACTIVITY_UNKNOWN;
  }

  bestScore = scores[0];

  for (i = 1U; i < count; ++i)
  {
    if (scores[i] > bestScore)
    {
      bestScore = scores[i];
      bestIndex = i;
    }
  }

  switch (bestIndex)
  {
    case 0U:
      return ACTIVITY_BIKING;

    case 1U:
      return ACTIVITY_WALKING;

    case 2U:
      return ACTIVITY_JOGGING;

    case 3U:
      return ACTIVITY_STATIONARY;

    default:
      return ACTIVITY_UNKNOWN;
  }
}

/**
  * @brief  Get printable activity name
  * @param  activity Activity enum
  * @retval String literal
  */
static const char *GetActivityName(ActivityClass_TypeDef activity)
{
  switch (activity)
  {
    case ACTIVITY_STATIONARY:
      return ACTIVITY_LABEL_STATIONARY;

    case ACTIVITY_WALKING:
      return ACTIVITY_LABEL_WALKING;

    case ACTIVITY_JOGGING:
      return ACTIVITY_LABEL_JOGGING;

    case ACTIVITY_BIKING:
      return ACTIVITY_LABEL_BIKING;

    default:
      return "Unknown";
  }
}

/**
  * @brief  Activity mode: collect one window and run the HAR model
  * @param  None
  * @retval None
  */
static void ApplyActivityMode(void)
{
  ActivitySampleLoopCount++;
  if (ActivitySampleLoopCount < ACTIVITY_SAMPLE_LOOPS)
  {
    return;
  }

  ActivitySampleLoopCount = 0U;
  BSP_ACCELERO_GetXYZ(AcceleroBuffer);

  ActivityWindow[ActivitySampleCount][0] = (float)AcceleroBuffer[0] * ACCEL_MG_TO_MS2;
  ActivityWindow[ActivitySampleCount][1] = (float)AcceleroBuffer[1] * ACCEL_MG_TO_MS2;
  ActivityWindow[ActivitySampleCount][2] = (float)AcceleroBuffer[2] * ACCEL_MG_TO_MS2;
  ActivitySampleCount++;

  if (ActivitySampleCount >= ACTIVITY_WINDOW_LEN)
  {
    float processedWindow[ACTIVITY_WINDOW_LEN][ACTIVITY_AXIS_NUM];

    PreprocessActivityWindow(ActivityWindow, processedWindow);

    if (aiRunInference(&processedWindow[0][0], STAI_NETWORK_IN_1_SIZE,
                       ActivityOutput, STAI_NETWORK_OUT_1_SIZE) == 0)
    {
      LastActivityClass = GetBestActivityClass(ActivityOutput, STAI_NETWORK_OUT_1_SIZE);
      SetActivityLed(LastActivityClass);
      printf("Activity: %s\r\n", GetActivityName(LastActivityClass));
    }
    else
    {
      printf("Activity inference failed.\r\n");
    }

    ActivitySampleCount = 0U;
  }
}


/**
  * @brief  HAL MSP callback for TIM4 PWM
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance != TIM4)
  {
    return;
  }

  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
