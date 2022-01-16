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

void API_FOC_Torque_Enable();
void API_FOC_Torque_Disable();

void API_FOC_Set_Torque_Flux_Currents_mA(float Iq_mA, float Id_mA);	// 	Setpoint Iq & Id

uint32_t API_FOC_Get_Timestamp_ms();
uint16_t API_FOC_Get_Timestamp_us();
float API_FOC_Get_Present_Torque_Current();
float API_FOC_Get_Present_Flux_Current();
float API_FOC_Get_Present_Voltage();
float API_FOC_Get_Present_Temp();
float API_FOC_Get_Processing_Time();
float API_FOC_Get_Processing_Frequency();

// low priority low frequency process, called by main loop()
void API_FOC_Service_Update()  __attribute__((section (".ccmram")));


#ifdef __cplusplus
}
#endif


#endif /* INC_FOC_H_ */
