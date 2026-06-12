#ifndef USER_APP_MODES_H
#define USER_APP_MODES_H

#include <stdint.h>
#include "app_config.h"

uint32_t ScaleTiltToDuty(int32_t axisMg);
void SetActivityLed(ActivityClass_TypeDef activity);
void ApplyDefaultMode(void);
void ApplyTiltMode(void);

#endif /* USER_APP_MODES_H */
