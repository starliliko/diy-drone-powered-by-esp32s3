/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * filter.h - Filtering functions
 */

#include <math.h>
#include <stdlib.h>

#include "filter.h"
#include "physicalConstants.h"

/**
 * IIR filter the samples.
 */
int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt)
{
  int32_t inScaled;
  int32_t filttmp = *filt;
  int16_t out;

  if (attenuation > (1<<IIR_SHIFT))
  {
    attenuation = (1<<IIR_SHIFT);
  }
  else if (attenuation < 1)
  {
    attenuation = 1;
  }

  // Shift to keep accuracy
  inScaled = in << IIR_SHIFT;
  // Calculate IIR filter
  filttmp = filttmp + (((inScaled-filttmp) >> IIR_SHIFT) * attenuation);
  // Scale and round
  out = (filttmp >> 8) + ((filttmp & (1 << (IIR_SHIFT - 1))) >> (IIR_SHIFT - 1));
  *filt = filttmp;

  return out;
}

/**
 * 2-Pole low pass filter
 */
void lpf2pInit(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
  if (lpfData == NULL || cutoff_freq <= 0.0f) {
    return;
  }

  lpf2pSetCutoffFreq(lpfData, sample_freq, cutoff_freq);
}

void lpf2pSetCutoffFreq(lpf2pData* lpfData, float sample_freq, float cutoff_freq)
{
  float fr = sample_freq/cutoff_freq;
  float ohm = tanf(M_PI_F/fr);
  float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm;
  lpfData->b0 = ohm*ohm/c;
  lpfData->b1 = 2.0f*lpfData->b0;
  lpfData->b2 = lpfData->b0;
  lpfData->a1 = 2.0f*(ohm*ohm-1.0f)/c;
  lpfData->a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
  lpfData->delay_element_1 = 0.0f;
  lpfData->delay_element_2 = 0.0f;
}

float lpf2pApply(lpf2pData* lpfData, float sample)
{
  float delay_element_0 = sample - lpfData->delay_element_1 * lpfData->a1 - lpfData->delay_element_2 * lpfData->a2;
  if (!isfinite(delay_element_0)) {
    // don't allow bad values to propigate via the filter
    delay_element_0 = sample;
  }

  float output = delay_element_0 * lpfData->b0 + lpfData->delay_element_1 * lpfData->b1 + lpfData->delay_element_2 * lpfData->b2;

  lpfData->delay_element_2 = lpfData->delay_element_1;
  lpfData->delay_element_1 = delay_element_0;
  return output;
}

float lpf2pReset(lpf2pData* lpfData, float sample)
{
  float dval = sample / (lpfData->b0 + lpfData->b1 + lpfData->b2);
  lpfData->delay_element_1 = dval;
  lpfData->delay_element_2 = dval;
  return lpf2pApply(lpfData, sample);
}

void notchFilterInit(lpf2pData* filterData, float sample_freq, float center_freq, float bandwidth_hz)
{
  if (filterData == NULL) {
    return;
  }

  notchFilterSet(filterData, sample_freq, center_freq, bandwidth_hz);
}

void notchFilterSet(lpf2pData* filterData, float sample_freq, float center_freq, float bandwidth_hz)
{
  if (filterData == NULL) {
    return;
  }

  if (sample_freq <= 0.0f || center_freq <= 0.0f || bandwidth_hz <= 0.0f) {
    filterData->b0 = 1.0f;
    filterData->b1 = 0.0f;
    filterData->b2 = 0.0f;
    filterData->a1 = 0.0f;
    filterData->a2 = 0.0f;
    filterData->delay_element_1 = 0.0f;
    filterData->delay_element_2 = 0.0f;
    return;
  }

  const float nyquist = sample_freq * 0.5f;
  if (center_freq >= nyquist) {
    filterData->b0 = 1.0f;
    filterData->b1 = 0.0f;
    filterData->b2 = 0.0f;
    filterData->a1 = 0.0f;
    filterData->a2 = 0.0f;
    filterData->delay_element_1 = 0.0f;
    filterData->delay_element_2 = 0.0f;
    return;
  }

  float q = center_freq / bandwidth_hz;
  if (q < 0.1f) {
    q = 0.1f;
  }

  const float w0 = 2.0f * M_PI_F * center_freq / sample_freq;
  const float alpha = sinf(w0) / (2.0f * q);
  const float cos_w0 = cosf(w0);

  const float b0 = 1.0f;
  const float b1 = -2.0f * cos_w0;
  const float b2 = 1.0f;
  const float a0 = 1.0f + alpha;
  const float a1 = -2.0f * cos_w0;
  const float a2 = 1.0f - alpha;

  filterData->b0 = b0 / a0;
  filterData->b1 = b1 / a0;
  filterData->b2 = b2 / a0;
  filterData->a1 = a1 / a0;
  filterData->a2 = a2 / a0;
  filterData->delay_element_1 = 0.0f;
  filterData->delay_element_2 = 0.0f;
}

float notchFilterApply(lpf2pData* filterData, float sample)
{
  return lpf2pApply(filterData, sample);
}

float notchFilterReset(lpf2pData* filterData, float sample)
{
  return lpf2pReset(filterData, sample);
}
