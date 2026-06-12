
#ifndef USER_APP_CONFIG_H
#define USER_APP_CONFIG_H

#include "stm32f4xx.h"
#include "app_x-cube-ai.h"
#include <stdint.h>
#include "los_typedef.h"

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

/* 全局变量声明（在 app_config.c 中定义） */
extern UINT32 LedControlTaskHandle;
extern TIM_HandleTypeDef Tim4Handle;
extern LedMode_TypeDef CurrentMode;
extern int16_t AcceleroBuffer[3];
extern float ActivityWindow[ACTIVITY_WINDOW_LEN][ACTIVITY_AXIS_NUM];
extern float ActivityOutput[STAI_NETWORK_OUT_1_SIZE];
extern uint32_t ActivitySampleCount;
extern uint32_t ActivitySampleLoopCount;
extern ActivityClass_TypeDef LastActivityClass;

/* 滤波器系数（在 app_config.c 中定义） */
extern const float ActivityFilterACoeff[AI_FILTER_ORDER + 1U];
extern const float ActivityFilterBCoeff[AI_FILTER_ORDER + 1U];
extern const float ActivityFilterInitialState[AI_FILTER_ORDER];

#endif /* USER_APP_CONFIG_H */
