/*
 * as5048a.c
 *
 *  Created on: Jan 15, 2021
 *      Author: Patrick
 */

#include "as5048a.h"
#include <math.h>
#include "math_tool.h"
#include "control_table.h"
#include "serial.h"


// serial communication (UART2) for TRACEs
// TODO : use STM32 CUBE MONITOR
extern HAL_Serial_Handler serial;

// µs TIMER
extern TIM_HandleTypeDef htim6;

// PWM IC TIMER
static TIM_HandleTypeDef * position_sensor_htim = 0;
static uint32_t calls = 0;

// position state
static uint32_t position_sensor_error = 0;
static uint32_t position_sensor_error_counter = 0;
static uint16_t present_time_us = 0;
static float present_position_rad = 0.0f;
static int16_t delta_t_us = 0;
static float delta_position_rad = 0.0f;
static float const bit_to_radians_ratio = M_2PI/4096.0f;
static float const max_radians = M_2PI/4096.0f*4095.0f;
// velocity state
static int16_t position_delta_time_us = 0;
static uint16_t last_position_time_us = 0;
static float last_position_rad = 0.0f;
static float present_velocity_rad = 0.0f;
// multi-turn position state
static int32_t present_revolution = 0;
static float present_position_multi_rad = 0.0f;

#define ALPHA_VELOCITY 0.25f // 0.25f default

void API_AS5048A_Position_Sensor_Init(TIM_HandleTypeDef * htim)
{
	position_sensor_htim = htim;
	HAL_TIM_IC_Start_IT(position_sensor_htim,TIM_CHANNEL_1); // Trigger It only when period is over.
	HAL_TIM_IC_Start(position_sensor_htim,TIM_CHANNEL_2); // CHANNEL 1 is PWM width and CHANNEL 2 is period
	HAL_Delay(3);
	present_revolution = 0;
	present_velocity_rad = 0.0f;
}

void API_AS5048A_Position_Sensor_It(TIM_HandleTypeDef *htim)
{
	if(htim==position_sensor_htim)
	{
		++calls;
		// timestamp as soon as possible
		present_time_us = __HAL_TIM_GET_COUNTER(&htim6);
		// capture the number of bits 1b
		// period is 4119 bits long with 8-bit trailer (value = 00000000b)
		// header is 16-bit long (12-bit init field (value=111111111111b), 4 for error field (value=1111b when OK))
		// when position is 0°, length is 16 bits
		// when position is MAX = 2*PI*(1-1/4096)°, length is 16+4095 bits
		// compute PWM width / PWM period * 4119bits that gives the number of 1 bits
		// @160MHz, CHANNEL1 = period = 53333 with PSC=2(+1)
		float const init_error_data_bits = 4119.0f*(float)__HAL_TIM_GET_COMPARE(position_sensor_htim,TIM_CHANNEL_2)/(float)__HAL_TIM_GET_COMPARE(position_sensor_htim,TIM_CHANNEL_1);
		// if data < 0 bits ==> must be an error
		if(init_error_data_bits<(16.0f-0.8f)) // add a 0.8 margin due to IC TIMER PRECISION and PWM precision
		{
			// set error
			position_sensor_error = 1;
			++position_sensor_error_counter;
			// Note : use the state when error (present time / position / velocity)
			// set encoder error
			regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR;
			//HAL_Serial_Print(&serial,"e");
		}
		else
		{
			// clear encoder error
			regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR);
			// reset error
			position_sensor_error = 0;
			// compute new position in radians and constrain it to [0..2pi[
			present_position_rad = ((init_error_data_bits-16.0f))*bit_to_radians_ratio;
			// limit [0,2PI[
			if(present_position_rad<0.0f)
				present_position_rad=0.0f;
			if(present_position_rad>max_radians)
				present_position_rad=max_radians;
			// delay since the last position
			position_delta_time_us = (int16_t)(present_time_us-last_position_time_us);

			// AS5048A specific
			// AS5048A specific
			// AS5048A specific
			// there is a zero crossing problem with AS5048A at high speed
			// we have to filter the actual position from an arbitrary base velocity ~ 1 RPM
			float const threshold_velocity_rds = M_2PI*1.0f; // Radians/s

			// compute the expected position according last position, the current velocity and the actual rate of position (~1ms)
			float const expected_position_rad = normalize_angle(last_position_rad+present_velocity_rad*position_delta_time_us/1000000.0f);

			// actual velocity is > threshold velocity ==> apply filter on actual position
			if(fabsf(present_velocity_rad)>threshold_velocity_rds)
			{
				// if actual position is near ZERO, use the expected position, ignore the actual position
				if( present_position_rad < 0.12f )
				{
					present_position_rad = expected_position_rad+0.25*difference_angle(present_position_rad,expected_position_rad);
				}
				// else ignore expected position, actual position is precise far from ZERO
			}

			// AS5048A specific
			// AS5048A specific
			// AS5048A specific

			// compute multi-turn position and velocity in radians
			delta_position_rad = present_position_rad-last_position_rad;
			if(delta_position_rad>M_PI)
			{
				--present_revolution;
				delta_position_rad-=M_2PI;
			}
			if(delta_position_rad<-M_PI)
			{
				++present_revolution;
				delta_position_rad+=M_2PI;
			}
			present_position_multi_rad = present_position_rad+(float)present_revolution*M_2PI;

			// compute velocity
			present_velocity_rad = ALPHA_VELOCITY * (delta_position_rad / (float)position_delta_time_us * 1000000.0f) + (1.0f-ALPHA_VELOCITY) * present_velocity_rad;


			// save last position
			last_position_time_us = present_time_us;
			last_position_rad = present_position_rad;
		}

	}
}

float API_AS5048A_Position_Sensor_Get_Radians_Estimation(uint16_t time_us)
{
	float result = 0.0f;
	delta_t_us = (int16_t)(time_us-present_time_us);
	// position has been received during FOC algorithm execution
	if(delta_t_us<0) // should never happend because of NVIC priority (TIM4 priority lower than ADC DMA priority)
	{
		// set encoder error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_TIMESTAMP;
		// return error
		result = 0.0f; // force ZERO
	}
	// check old sample error
	else if(delta_t_us>1200) //1.2ms
	{
		// set encoder error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_NOT_RESPONDING;
		// return error
		result = 0.0f; // force ZERO
	}
	// normal
	else
	{
		// clear encoder error
		regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_POSITION_SENSOR_NOT_RESPONDING);
		// compute estimation
		result = present_position_rad + present_velocity_rad*((float)delta_t_us+(float)position_delta_time_us)/1000000.0f;
	}
	return result;
}

float API_AS5048A_Position_Sensor_Get_Radians()
{
	return present_position_rad;
}

float API_AS5048A_Position_Sensor_Get_Multiturn_Radians()
{
	return present_position_multi_rad;
}

float API_AS5048A_Position_Sensor_Get_RPS()
{
	return present_velocity_rad;
}

float API_AS5048A_Position_Sensor_Get_DPS()
{
	return RADIANS_TO_DEGREES(present_velocity_rad);
}

uint16_t API_AS5048A_Position_Sensor_Get_Timestamp()
{
	return present_time_us;
}

uint16_t API_AS5048A_Position_Sensor_Get_DeltaTimestamp()
{
	return position_delta_time_us;
}

int16_t API_AS5048A_Position_Sensor_Get_DeltaTimeEstimation()
{
	return delta_t_us;
}

uint32_t API_AS5048A_Position_Sensor_Error()
{
	return position_sensor_error;
}

uint32_t API_AS5048A_Position_Sensor_Error_Counter()
{
	return position_sensor_error_counter;
}

float API_AS5048A_Position_Sensor_Get_DeltaRad()
{
	return delta_position_rad;
}

uint32_t API_AS5048A_Position_Sensor_It_Counter()
{
	return calls;
}
