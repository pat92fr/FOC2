/*
 * math_tool.c
 *
 *  Created on: Nov 6, 2020
 *      Author: Patrick, Kai
 */

#include "math_tool.h"
#include<math.h>

int32_t constrain(int32_t x, int32_t min, int32_t max)
{
    if(x<min) return min;
    if(x>max) return max;
    return x;
}

float fconstrain(float x, float min, float max)
{
    if(x<min) return min;
    if(x>max) return max;
    return x;
}

float fconstrain_both(float x, float abs)
{
    if(x<-abs) return -abs;
    if(x>abs)  return abs;
    return x;
}

float mfmod(float x,float y)
{
	float a = x/y;
	return (a-(int)a)*y;
}

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    if(x<=in_min) return out_min;
    if(x>=in_max) return out_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
    if(x<=in_min) return out_min;
    if(x>=in_max) return out_max;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float normalize_angle(float angle_rad)
{
	float const a = mfmod(angle_rad, M_2PI);
	return a >= 0.0f ? a : (a + M_2PI);
}

float difference_angle(float a_rad, float b_rad)
{
	float delta = a_rad-b_rad;
	if(delta>M_PI)
	{
		return delta-M_2PI;
	}
	if(delta<=-M_PI)
	{
		return delta+M_2PI;
	}
	return delta;
}
