#include "app_config.h"

UINT32 LedControlTaskHandle;
TIM_HandleTypeDef Tim4Handle;
LedMode_TypeDef CurrentMode = LED_MODE_DEFAULT;
int16_t AcceleroBuffer[3];
float ActivityWindow[ACTIVITY_WINDOW_LEN][ACTIVITY_AXIS_NUM];
float ActivityOutput[STAI_NETWORK_OUT_1_SIZE];
uint32_t ActivitySampleCount = 0U;
uint32_t ActivitySampleLoopCount = 0U;
ActivityClass_TypeDef LastActivityClass = ACTIVITY_UNKNOWN;

const float ActivityFilterACoeff[AI_FILTER_ORDER + 1U] = {
  1.0f, -3.868656635f, 5.614526749f, -3.622760773f, 0.8768966198f
};
const float ActivityFilterBCoeff[AI_FILTER_ORDER + 1U] = {
  0.9364275932f, -3.745710373f, 5.618565559f, -3.745710373f, 0.9364275932f
};
const float ActivityFilterInitialState[AI_FILTER_ORDER] = {
  -0.936528250873f, 2.809571532101f, -2.809559172096f, 0.936515859573f
};
