#ifndef USER_APP_ACTIVITY_H
#define USER_APP_ACTIVITY_H

#include <stdint.h>
#include "app_config.h"

void ResetActivityWindow(void);
void HighPassFilterAxis(const float *input, float *output, uint32_t length);
void PreprocessActivityWindow(float input[][ACTIVITY_AXIS_NUM], float output[][ACTIVITY_AXIS_NUM]);
ActivityClass_TypeDef GetBestActivityClass(const float *scores, uint32_t count);
const char *GetActivityName(ActivityClass_TypeDef activity);
void ApplyActivityMode(void);

#endif /* USER_APP_ACTIVITY_H */
