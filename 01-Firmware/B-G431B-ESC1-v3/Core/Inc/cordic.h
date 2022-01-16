/*
 * mycordic.h
 *
 *  Created on: Jan 14, 2021
 *      Author: Patrick
 */

#ifndef INC_CORDIC_H_
#define INC_CORDIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "math_tool.h"

#include <math.h>

// CORDIC
extern CORDIC_HandleTypeDef hcordic;

// source : MJBot https://github.com/mjbots/moteus/blob/main/fw/math.h
int32_t RadiansToQ31(float) __attribute__((always_inline));

inline int32_t RadiansToQ31(float x)
{
  // First we scale, then wrap, and finally convert out.
  const float scaled = x / M_2PI;
  // Now we wrap to be from 0 to 1.
  const int32_t i = (int32_t)(scaled);
  float mod = scaled - i;
  if (mod < 0) { mod += 1.0f; }

  return (int32_t)(((mod > 0.5f) ? (mod - 1.0f) : mod) * 4294967296.0f);
}

float Q31ToRadians(int32_t) __attribute__((always_inline));

inline float Q31ToRadians(int32_t x)
{
  return (float)(x)/2147483648.0f;
}

HAL_StatusTypeDef API_CORDIC_Processor_Init()
{
	CORDIC_ConfigTypeDef config = {
			CORDIC_FUNCTION_COSINE, // ouput : cosine, then sine
			CORDIC_SCALE_0, // not used
			CORDIC_INSIZE_32BITS, // q31
			CORDIC_OUTSIZE_32BITS, // q31
			CORDIC_NBWRITE_1, // ARG2 is 1 default
			CORDIC_NBREAD_2, // read cosine and sine
			CORDIC_PRECISION_5CYCLES // better than 10-3
	};
	return HAL_CORDIC_Configure(&hcordic, &config);
}

HAL_StatusTypeDef API_CORDIC_Processor_Update(float theta_rad, float * c, float * s) __attribute__((always_inline));

inline HAL_StatusTypeDef API_CORDIC_Processor_Update(float theta_rad, float * c, float * s)
{
	int32_t InBuff = RadiansToQ31(theta_rad);
	int32_t OutBuff[2] = {0,0};
	HAL_StatusTypeDef result = HAL_CORDIC_Calculate(&hcordic,&InBuff,OutBuff,1,0);
	if(HAL_OK==result)
	{
		*c = Q31ToRadians(OutBuff[0]);
		*s = Q31ToRadians(OutBuff[1]);
	}
	else
	{
		//regs[REG_PROTOCOL_CRC_FAIL]++; // DEBUG
		// TODO hardware error !
		// TODO hardware error !
		// TODO hardware error !
		// TODO hardware error !
	}
	return result;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_CORDIC_H_ */
