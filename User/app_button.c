#include "app_button.h"
#include "app_config.h"
#include "stm32f411e_discovery.h"

uint8_t PollButtonPressedEvent(void)
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
