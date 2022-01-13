/*
 * foc_utils.c
 *
 *  Created on: 13 janv. 2022
 *      Author: Patrick
 */


#include "foc_utils.h"
#include "math_tool.h"
#include "control_table.h"

// hard-coded settings
#define MAX_PWM_DUTY_CYCLE 			0.95f 	// %
#define MIN_PWM_DUTY_CYCLE 			0.05f 	// %
	// with a PWM frequency (20Khz and more), deadtime must be taken in account.
	// CUBEMX configuration : deadtime = 128 at f=160MHz ==> deadtime = 800ns
	// MIN/MAX DUTY CYCLE is set in order to allow current sense when TIM1 update event triggered (800ns is about 2% PWM at 20KHz)
#define CSVPWM 					 	// uncomment to use CSVPWM (conventional space vector pulse width modulation),


// peripherals
extern TIM_HandleTypeDef htim1;

void LL_FOC_Inverse_Clarke_Park_PWM_Generation( float Vd, float Vq, float cosine_theta, float sine_theta, float present_voltage_V )
{
	// Inverse Park Transformation
	float const Valpha = Vd * cosine_theta - Vq * sine_theta;
	float const Vbeta  = Vq * cosine_theta + Vd * sine_theta;
	// Inverse Clarke Transformation
	float const Va = Valpha;
	float const Vb = -0.5f * Valpha + SQRT3_2 * Vbeta;
	float const Vc = -0.5f  *Valpha - SQRT3_2 * Vbeta;
	// apply CSVPWM to (Va,Vb,Vc)
	float const Vneutral = 0.5f*(fmaxf(fmaxf(Va,Vb),Vc)+fminf(fminf(Va,Vb),Vc));
	// convert (Va,Vb,Vc) [-max_voltage_V/2,max_voltage_V/2] to PWM duty cycles % [MIN_PWM_DUTY_CYCLE MAX_PWM_DUTY_CYCLE]
	float const duty_cycle_PWMa = fconstrain((Va-Vneutral)/present_voltage_V+0.5f,MIN_PWM_DUTY_CYCLE,MAX_PWM_DUTY_CYCLE);
	float const duty_cycle_PWMb = fconstrain((Vb-Vneutral)/present_voltage_V+0.5f,MIN_PWM_DUTY_CYCLE,MAX_PWM_DUTY_CYCLE);
	float const duty_cycle_PWMc = fconstrain((Vc-Vneutral)/present_voltage_V+0.5f,MIN_PWM_DUTY_CYCLE,MAX_PWM_DUTY_CYCLE);
	// update TIMER CCR registers and apply BRAKE in case of hardware failure or control mode desactivated
	if( (regs[REG_HARDWARE_ERROR_STATUS] != 0) && (regs[REG_CONTROL_MODE] == REG_CONTROL_MODE_IDLE) )
	{
		// compute a valid BRAKE value
		uint16_t const CCRx = (uint16_t)(0.5f*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))-1; // note : 0 is OK too
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCRx);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CCRx);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CCRx);
	}
	else
	{
		// convert PWM duty cycles % to TIMER1 CCR register values
		// fPWM = 22KHz
		// fTIM = 160MHz
		// in PWM centered mode, for the finest possible resolution :
		// ARR = fTIM/(2 * fPWM) -1 => ARR = 3635
		uint16_t const CCRa = (uint16_t)(duty_cycle_PWMa*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))-1;
		uint16_t const CCRb = (uint16_t)(duty_cycle_PWMb*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))-1;
		uint16_t const CCRc = (uint16_t)(duty_cycle_PWMc*(float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1))-1;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,CCRa);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,CCRb);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,CCRc);
	}
}
