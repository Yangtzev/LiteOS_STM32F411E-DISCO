#include "app_activity.h"
#include "app_config.h"
#include "app_modes.h"
#include "stm32f411e_discovery_accelerometer.h"
#include <math.h>
#include <string.h>

void ResetActivityWindow(void)
{
  memset(ActivityWindow, 0, sizeof(ActivityWindow));
  memset(ActivityOutput, 0, sizeof(ActivityOutput));
  ActivitySampleCount = 0U;
  ActivitySampleLoopCount = 0U;
  LastActivityClass = ACTIVITY_UNKNOWN;
}

void HighPassFilterAxis(const float *input, float *output, uint32_t length)
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

void PreprocessActivityWindow(float input[][ACTIVITY_AXIS_NUM],
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

  /* Debug: print first raw sample (scaled x1000 to avoid float-only printf) */
  printf("[DEBUG] Preprocess start raw0_milli=[%d,%d,%d]\r\n",
         (int)(axisInput[0][0] * 1000.0f),
         (int)(axisInput[1][0] * 1000.0f),
         (int)(axisInput[2][0] * 1000.0f));

  for (axis = 0U; axis < ACTIVITY_AXIS_NUM; ++axis)
  {
    HighPassFilterAxis(axisInput[axis], axisDyn[axis], ACTIVITY_WINDOW_LEN);
  }

  /* Debug: show first dynamic sample after HPF */
  printf("[DEBUG] HighPass done dyn0_milli=[%d,%d,%d]\r\n",
         (int)(axisDyn[0][0] * 1000.0f),
         (int)(axisDyn[1][0] * 1000.0f),
         (int)(axisDyn[2][0] * 1000.0f));

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

  /* Debug: show first processed sample */
  printf("[DEBUG] Preprocess done proc0_milli=[%d,%d,%d]\r\n",
         (int)(output[0][0] * 1000.0f),
         (int)(output[0][1] * 1000.0f),
         (int)(output[0][2] * 1000.0f));
}

ActivityClass_TypeDef GetBestActivityClass(const float *scores, uint32_t count)
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

const char *GetActivityName(ActivityClass_TypeDef activity)
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

void ApplyActivityMode(void)
{
  uint32_t i;
  ActivitySampleLoopCount++;
  if (ActivitySampleLoopCount < ACTIVITY_SAMPLE_LOOPS)
  {
    return;
  }

  ActivitySampleLoopCount = 0U;
  BSP_ACCELERO_GetXYZ(AcceleroBuffer);

    /* Debug: raw accelerometer reading (mg) */
    printf("[DEBUG] Sample read raw_mg=[%d,%d,%d]\r\n",
      (int)AcceleroBuffer[0], (int)AcceleroBuffer[1], (int)AcceleroBuffer[2]);

  ActivityWindow[ActivitySampleCount][0] = (float)AcceleroBuffer[0] * ACCEL_MG_TO_MS2;
  ActivityWindow[ActivitySampleCount][1] = (float)AcceleroBuffer[1] * ACCEL_MG_TO_MS2;
  ActivityWindow[ActivitySampleCount][2] = (float)AcceleroBuffer[2] * ACCEL_MG_TO_MS2;
    printf("[DEBUG] Window[%u] appended (mg)=[%d,%d,%d]\r\n",
      ActivitySampleCount,
      (int)AcceleroBuffer[0], (int)AcceleroBuffer[1], (int)AcceleroBuffer[2]);
    ActivitySampleCount++;

  if (ActivitySampleCount >= ACTIVITY_WINDOW_LEN)
  {
    float processedWindow[ACTIVITY_WINDOW_LEN][ACTIVITY_AXIS_NUM];
    printf("[DEBUG] Window full: start preprocess and inference\r\n");
    PreprocessActivityWindow(ActivityWindow, processedWindow);

    printf("[DEBUG] Calling aiRunInference...\r\n");
    if (aiRunInference(&processedWindow[0][0], STAI_NETWORK_IN_1_SIZE,
                       ActivityOutput, STAI_NETWORK_OUT_1_SIZE) == 0)
    {
      /* Debug: print confidence scores as integer percentages */
      for (i = 0U; (i < STAI_NETWORK_OUT_1_SIZE) && (i < 8U); ++i)
      {
        printf("[DEBUG] score[%u]=%d%%\r\n", i, (int)(ActivityOutput[i] * 100.0f));
      }

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
