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

#define sensorType_AS5600_I2C 1
#define sensorType_AS5048A_PWM 2

int positionSensor_init(uint8_t sensorType);
float positionSensor_getRadiansEstimation(uint16_t time_us);
void positionSensor_update(void);
float positionSensor_getRadians(void);
float positionSensor_getRadiansMultiturn(void);
float positionSensor_getDegree(void);
float positionSensor_getDegreeMultiturn(void);
float positionSensor_getVelocityDegree(void);
uint8_t positionSensor_getType(void);

#endif /* INC_POSITION_SENSOR_H_ */
