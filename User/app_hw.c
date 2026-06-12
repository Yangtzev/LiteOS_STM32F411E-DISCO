#include "app_hw.h"
#include "app_config.h"
#include "stm32f4xx_hal.h"

/**
 * Configure TIM4 for PWM on four LED pins
 */
void TIM4_Config(void)
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

uint32_t GetTim4Clock(void)
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

void SetLedDuty(uint32_t channel, uint32_t duty)
{
  if (duty > LED_PWM_PERIOD)
  {
    duty = LED_PWM_PERIOD;
  }

  __HAL_TIM_SET_COMPARE(&Tim4Handle, channel, duty);
}

void ClearAllLeds(void)
{
  SetLedDuty(TIM_CHANNEL_1, 0U);
  SetLedDuty(TIM_CHANNEL_2, 0U);
  SetLedDuty(TIM_CHANNEL_3, 0U);
  SetLedDuty(TIM_CHANNEL_4, 0U);
}

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
