/*
 * foc.h
 *
 *  Created on: 15 janv. 2021
 *      Author: Patrick, Kai
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void API_FOC_Init();

int API_FOC_Calibrate();

// low priority low frequency process
void API_FOC_Service_Update();

// high priority high frequency process
void API_FOC_Torque_Update(
		float setpoint_torque_current_mA,
		float setpoint_flux_current_mA,
		float setpoint_velocity_dps
); //  __attribute__((section (".ccmram")));

void API_FOC_Set_Flux_Angle(
		float setpoint_electrical_angle_rad,
		float setpoint_flux_voltage_V
);

void API_FOC_Set_Flux_Velocity(
		uint16_t present_time_us,
		float setpoint_electrical_velocity_dps,
		float setpoint_flux_voltage_V
);

float API_FOC_Get_Present_Torque_Current();
float API_FOC_Get_Present_Flux_Current();
float API_FOC_Get_Present_Voltage();
float API_FOC_Get_Present_Temp();
float API_FOC_Get_Processing_Time();
float API_FOC_Get_Processing_Frequency();

void API_FOC_It(ADC_HandleTypeDef *hadc);// __attribute__((section (".ccmram")));

#ifdef __cplusplus
}
#endif


#endif /* INC_FOC_H_ */
