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
void API_FOC_Reset();

// low priority low frequency process
void API_FOC_Service_Update()  __attribute__((section (".ccmram")));

// high priority high frequency process
void API_FOC_Torque_Update()  __attribute__((section (".ccmram")));

float API_FOC_Get_Present_Torque_Current();
float API_FOC_Get_Present_Flux_Current();
float API_FOC_Get_Present_Voltage();
float API_FOC_Get_Present_Temp();
float API_FOC_Get_Processing_Time();
float API_FOC_Get_Processing_Frequency();

void API_FOC_It(ADC_HandleTypeDef *hadc) __attribute__((section (".ccmram")));

#ifdef __cplusplus
}
#endif


#endif /* INC_FOC_H_ */
