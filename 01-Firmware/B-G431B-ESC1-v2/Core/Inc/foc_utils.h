/*
 * foc_utils.h
 *
 *  Created on: 13 janv. 2022
 *      Author: Patrick
 */

#ifndef INC_FOC_UTILS_H_
#define INC_FOC_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

// low level function
// this function checks REG_HARDWARE_ERROR_STATUS register and enforce BRAKE is register not null
// this function use the present_voltage_V state variable to adjust PWM duty cycle according power supply voltage
void LL_FOC_Inverse_Clarke_Park_PWM_Generation( float Vd, float Vq, float cosine_theta, float sine_theta, float present_voltage_V ) __attribute__((section (".ccmram")));

#ifdef __cplusplus
}
#endif


#endif /* INC_FOC_UTILS_H_ */
