/*
 * math_tool.h
 *
 *  Created on: 4 nov. 2020
 *      Author: Patrick, Kai
 */

#ifndef INC_MATH_TOOL_H_
#define INC_MATH_TOOL_H_

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

#define M_2PI (6.283185307179586f)
#define M_3PI_2 (4.7123889803846f)

#define INV_SQRT3  	(0.5773502691896257f)
#define SQRT3 		(1.7320508075688772f)

// TODO CCRAM for function called from FOC
// TODO CCRAM for function called from FOC
// TODO CCRAM for function called from FOC
// TODO CCRAM for function called from FOC
// TODO CCRAM for function called from FOC

int32_t constrain(int32_t x, int32_t min, int32_t max);
float fconstrain(float x, float min, float max) __attribute__((section (".ccmram")));
float fconstrain_both(float x, float abs);

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);

inline double mfmod(float x,float y)
{
	float a = x/y;
	return (a-(int)a)*y;
}

#define RADIANS_TO_DEGREES(rad) ((rad)*57.2957795130823208767981548f)
#define DEGREES_TO_RADIANS(deg) ((deg)*0.01745329251994329576923690f)

// normalizing radian angle to [0,2PI]
float normalize_angle(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* INC_MATH_TOOL_H_ */
