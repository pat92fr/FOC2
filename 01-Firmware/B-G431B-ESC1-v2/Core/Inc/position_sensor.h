/*
 * position_sensor.h
 *
 *  Created on: 08.03.2021
 *      Author: kai
 */

#ifndef INC_POSITION_SENSOR_H_
#define INC_POSITION_SENSOR_H_

#include "as5048a.h"
#include "as5600.h"

typedef enum
{
	AS5600_I2C,
	AS5048A_PWM
} e_sensor_type;

int positionSensor_init(e_sensor_type sensor_type);
float positionSensor_getRadiansEstimation(uint16_t time_us) __attribute__((section (".ccmram")));;
void positionSensor_update(void);
float positionSensor_getRadians(void);
float positionSensor_getRadiansMultiturn(void);
float positionSensor_getDegree(void);
float positionSensor_getDegreeMultiturn(void);
float positionSensor_getVelocityDegree(void);
e_sensor_type positionSensor_getType(void);
uint16_t positionSensor_getDeltaTimestamp();
int16_t positionSensor_getDeltaTimeEstimation();

#endif /* INC_POSITION_SENSOR_H_ */
