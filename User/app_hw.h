#ifndef USER_APP_HW_H
#define USER_APP_HW_H

#include "stm32f4xx.h"
#include <stdint.h>

void TIM4_Config(void);
uint32_t GetTim4Clock(void);
void SetLedDuty(uint32_t channel, uint32_t duty);
void ClearAllLeds(void);

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);

#endif /* USER_APP_HW_H */
