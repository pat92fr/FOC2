/*
 * position_sensor.c
 *
 *  Created on: 08.03.2021
 *      Author: kai
 */
#include <stdio.h>
#include <stdlib.h>
#include "position_sensor.h"
#include "as5600.h"
#include "as5048a.h"
#include "math_tool.h"
#include "control_table.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim4;

// µs TIMER
extern TIM_HandleTypeDef htim6;

typedef struct {

	e_sensor_type sensor_type;

	// AS5600 handle
	AS5600_TypeDef *as5600Handle;

	// common sensor values
	uint16_t lastUpdate;
    uint16_t last_angle_data;
    float velocity_deg;
	float velocity_rad;

	float angle_rad;
	float angle_deg;

    float angle_prev_rad;
    float angle_prev_deg;

    int natural_direction;
    float full_rotation_offset;

} positionSensor_t;

static positionSensor_t* sensor;

positionSensor_t *positionSensor_New(void) {
	positionSensor_t *sensor = (positionSensor_t *)calloc(1, sizeof(positionSensor_t));
    return sensor;
}

int positionSensor_init(e_sensor_type sensor_type)
{

	int status=0;

	sensor = positionSensor_New();

	switch(sensor_type)
	{
	case AS5600_I2C:
	{
		uint16_t angle_data;

		sensor->full_rotation_offset =0;

		sensor->as5600Handle = AS5600_New();
		sensor->as5600Handle->i2cHandle=&hi2c1;
		AS5600_Init(sensor->as5600Handle);

		AS5600_GetAngle(sensor->as5600Handle, &angle_data);
		sensor->last_angle_data = angle_data;

		// velocity calculation init
		sensor->angle_rad = 0;
		sensor->lastUpdate = __HAL_TIM_GET_COUNTER(&htim6);

		sensor->sensor_type = sensor_type;

		status = 1;
	    break;
	}
	case AS5048A_PWM:
		API_AS5048A_Position_Sensor_Init(&htim4);
		sensor->sensor_type = sensor_type;
		status = 1;
	    break;
	default:
		status = 0;
		break;
	}
	return status;
}

float positionSensor_getRadiansEstimation(uint16_t time_us)
{
	switch(sensor->sensor_type)
	{
	case AS5600_I2C:
	{
		uint16_t delta_t_us;
		if (time_us <= sensor->lastUpdate) {
			delta_t_us = 0xffff-sensor->lastUpdate+time_us;
		}else{
			delta_t_us = (time_us - sensor->lastUpdate);
		}


		// position has been received during FOC algorithm execution
		if(delta_t_us<0) // should never happend because of NVIC priority (TIM4 priority lower than ADC DMA priority)
		{
			// set encoder error
			regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_TIMESTAMP;
			// return error
			return 0.0f; // force ZERO
		}
		// check old sample error
		else if(delta_t_us>1200) //1.2ms
		{
			// set encoder error
			regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_NOT_RESPONDING;
			// return error
			return 0.0f; // force ZERO
		}
		// normal
		else
		{
			// clear encoder error
			regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_POSITION_SENSOR_NOT_RESPONDING);
			// compute estimation
			return sensor->angle_rad + sensor->velocity_rad*(float)(delta_t_us+200)/1000000.0f; // Slow filter latency is 0.2ms
		}
	}
	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_Radians_Estimation(time_us);
	default:
		return 0;
	}
}

//  Shaft angle calculation
//  angle is in radians [rad]
void positionSensor_getAngle(void){
  // raw data from the sensor
  uint16_t angle_data;

  float cpr = pow(2, 12);
  AS5600_GetAngle(sensor->as5600Handle, &angle_data);

  // tracking the number of rotations
  // in order to expand angle range form [0,2PI]
  // to basically infinity
  float d_angle = (float)(angle_data - sensor->last_angle_data);
  // if overflow happened track it as full rotation
  if(abs(d_angle) > (0.8*cpr) ) sensor->full_rotation_offset += d_angle > 0 ? -M_2PI : M_2PI;
  // save the current angle value for the next steps
  // in order to know if overflow happened
  sensor->last_angle_data = angle_data;

  // return the full angle
  // (number of full rotations)*2PI + current sensor angle
   sensor->angle_rad = sensor->full_rotation_offset + ( (float)angle_data / (float)cpr) * M_2PI;
   sensor->angle_deg = RADIANS_TO_DEGREES(sensor->angle_rad);

}

void positionSensor_update(void){


	switch(sensor->sensor_type)
	{
	// get new data from as5600 sensor
	case AS5600_I2C:
	{
		float delta_time_us;
		float d_angle;
		uint16_t angle_data;
		float cpr = pow(2, 12);

		// calculate sample time
		uint16_t now_us = __HAL_TIM_GET_COUNTER(&htim6);
		if (now_us <= sensor->lastUpdate) {
			delta_time_us = 0xffff-sensor->lastUpdate+now_us;
		}else{
			delta_time_us = (now_us - sensor->lastUpdate);
		}

		// raw data from the sensor
		HAL_StatusTypeDef status = AS5600_GetAngle(sensor->as5600Handle, &angle_data);

		if(status!=HAL_OK)
		{	// set encoder error
			regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR;
			regs[REG_PROTOCOL_CRC_FAIL]++;
		}

		// tracking the number of rotations
		// in order to expand angle range form [0,2PI]
		// to basically infinity
		d_angle = (float)(angle_data - sensor->last_angle_data);
		// if overflow happened track it as full rotation
		if(abs(d_angle) > (0.8*cpr) ) sensor->full_rotation_offset += d_angle > 0 ? -M_2PI : M_2PI;
		// save the current angle value for the next steps
		// in order to know if overflow happened
		sensor->last_angle_data = angle_data;

		// return the full angle
		// (number of full rotations)*2PI + current sensor angle
		sensor->angle_rad = sensor->full_rotation_offset + ( (float)angle_data / (float)cpr) * M_2PI;
		sensor->angle_deg = RADIANS_TO_DEGREES(sensor->angle_rad);

		// velocity calculation
		sensor->velocity_rad = ((sensor->angle_rad - sensor->angle_prev_rad)/delta_time_us* 1000000.0f);
		sensor->velocity_deg = ((sensor->angle_deg - sensor->angle_prev_deg)/delta_time_us* 1000000.0f);

		// last angle
		sensor->angle_prev_rad = sensor->angle_rad;
		sensor->angle_prev_deg = sensor->angle_deg;

		sensor->lastUpdate = __HAL_TIM_GET_COUNTER(&htim6);
		break;
	}
	default:
        break;
    }
}

float positionSensor_getRadians(void)
{
	switch(sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->angle_rad;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_Radians();

	default:
		return 0;
	}
}

float positionSensor_getRadiansMultiturn(void)
{
	switch(sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->angle_rad;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_Multiturn_Radians();

	default:
		return 0;
	}
}

float positionSensor_getDegree(void){
	switch(sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->angle_deg;

	case AS5048A_PWM:
		return RADIANS_TO_DEGREES(API_AS5048A_Position_Sensor_Get_Radians());

	default:
		return 0;
	}
}

float positionSensor_getDegreeMultiturn(void){
	switch(sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->angle_deg;

	case AS5048A_PWM:
		return RADIANS_TO_DEGREES(API_AS5048A_Position_Sensor_Get_Multiturn_Radians());

	default:
		return 0;
	}
}

float positionSensor_getVelocityDegree(void){
	switch(sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->velocity_deg;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_DPS();

	default:
		return 0;
	}
}



e_sensor_type positionSensor_getType(void)
{
	return sensor->sensor_type;
}


uint16_t positionSensor_getDeltaTimestamp()
{
	switch(sensor->sensor_type)
	{
	case AS5600_I2C:
		return 0;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_DeltaTimestamp();

	default:
		return 0;
	}
}

int16_t positionSensor_getDeltaTimeEstimation()
{
	switch(sensor->sensor_type)
	{
	case AS5600_I2C:
		return 0;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_DeltaTimeEstimation();

	default:
		return 0;
	}
}
