#include "app_modes.h"
#include "app_hw.h"
#include "stm32f411e_discovery_accelerometer.h"

uint32_t ScaleTiltToDuty(int32_t axisMg)
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

void SetActivityLed(ActivityClass_TypeDef activity)
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

/* ClearAllLeds implemented in app_hw.c */

void ApplyDefaultMode(void)
{
  SetLedDuty(TIM_CHANNEL_1, LED_PWM_PERIOD);
  SetLedDuty(TIM_CHANNEL_2, LED_PWM_PERIOD);
  SetLedDuty(TIM_CHANNEL_3, LED_PWM_PERIOD);
  SetLedDuty(TIM_CHANNEL_4, LED_PWM_PERIOD);
}

void ApplyTiltMode(void)
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
