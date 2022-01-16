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

float fconstrain(float x, float min, float max) __attribute__((section (".ccmram")));
float mfmod(float x,float y) __attribute__((section (".ccmram")));

#define RADIANS_TO_DEGREES(rad) ((rad)*57.2957795130823208767981548f)
#define DEGREES_TO_RADIANS(deg) ((deg)*0.01745329251994329576923690f)

// normalizing radian angle to [0,2PI]
float normalize_angle(float angle_rad) __attribute__((section (".ccmram")));
float difference_angle(float a, float b) __attribute__((section (".ccmram")));

#ifdef __cplusplus
}
#endif

#endif /* INC_MATH_TOOL_H_ */
