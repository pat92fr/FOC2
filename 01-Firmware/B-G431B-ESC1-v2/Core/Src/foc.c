/*
 * foc.c
 *
 *  Created on: 16 janv. 2021
 *      Author: Patrick, Kai
 */

/// DOC FOC(open loop) : https://docs.simplefoc.com/foc_theory
/// DOC SVM https://www.embedded.com/painless-mcu-implementation-of-space-vector-modulation-for-electric-motor-systems/

#include "foc.h"
#include "foc_utils.h"
#include "cordic.h"
#include "serial.h"
#include "position_sensor.h"
#include "math_tool.h"
#include "control_table.h"
#include "binary_tool.h"
#include "pid.h"

#include <string.h>
#include <math.h>

// hard-coded settings
#define ALPHA_CURRENT_SENSE_OFFSET	0.001f 	// low pass filter for calibrating the phase current ADC offset (automatically)

// peripherals
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

// serial communication (UART2) for TRACEs
// TODO : use STM32 CUBE MONITOR
extern HAL_Serial_Handler serial;
extern float setpoint_torque_current_mA;
extern float setpoint_flux_current_mA;
float setpoint_electrical_angle_rad = 0.0f;
float setpoint_flux_voltage_V = 0.0f;

// FOC private variables
static int32_t current_samples = 0;
volatile uint16_t ADC1_DMA[5] = { 0,0,0,0,0 }; 	// Dummy conversion (ST workaround for -x),
volatile uint16_t ADC2_DMA[3] = { 0,0,0 }; 		// Dummy conversion (ST workaround for -x)
static uint16_t motor_current_input_adc[3] = {0.0f,0.0f,0.0f};
static float motor_current_input_adc_offset[3] = {2464.0f,2482.0f,2485.0f};
static float motor_current_input_adc_KmA = -29.41f; // V/mA // note : the (-) sign here
// process phase current
// Note : when current flows inward phase, shunt voltage is negative
// Note : when current flows outward phase, shunt voltage is positive
// Note : The current sign is positive when flowing in to a phase
// Note : The current sign is negative when flowing out from a phase

static float motor_current_mA[3] = {0.0f,0.0f,0.0f};
static float present_Id_mA = 0.0;
static float present_Iq_mA = 0.0f;
static pid_context_t flux_pi;
static pid_context_t torque_pi;

// foc analog measure
static float potentiometer_input_adc = 0.0f;
static float vbus_input_adc = 0.0f;
static float temperature_input_adc = 0.0f;
static float present_voltage_V = 0.0f;
static float present_temperature_C = 0.0f;

// foc performance monitoring (public)
static float average_processing_time_us = 0.0f;
static uint32_t foc_counter = 0;

void LL_FOC_Update_Temperature() __attribute__((section (".ccmram")));
void LL_FOC_Update_Voltage() __attribute__((section (".ccmram")));

// user API function
// this function reset state of FOC
// This function starts peripherals
void API_FOC_Init()
{
	// Motor PWM init and BRAKE
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1) ;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2) ;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3) ;
	// OPAMP and ADC init
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_DMA,5);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_DMA,3);
	// CORDIC init
	API_CORDIC_Processor_Init();
	// Reset state
	API_FOC_Reset();
}

// user API function
// this function reset state of FOC
void API_FOC_Reset()
{
	pid_reset(&flux_pi);
	pid_reset(&torque_pi);
}

// low level function
// this function update present_temperature_C
// this function update REG_HARDWARE_ERROR_STATUS register (set/reset HW_ERROR_BIT_OVERHEATING bit)
// this function use REG_TEMPERATURE_LIMIT register
// the temperature ADC samples are collected with phase current samples
void LL_FOC_Update_Temperature()
{
	// convert ADC sample into temperature (STM32G431-ESC1 specific)
	static float const R60 = 4700.0f; // ohm
	static float const eps = 0.1f; // epsilon (avoid divide by zero)
	float const R_NTC = R60*(4096.0f/(temperature_input_adc+eps)-1.0f); // 10kohm NTC at 25°C
	static float const Beta = 3455.0f; // for a 10k NTC
	static float const Kelvin = 273.15f; //°C
	static float const T0 = 273.15f + 25.0f;
	static float const R0 = 10000.0f; // 10kohm at 25° for 10k NTC
	float const present_temperature_K = Beta * T0 / ( Beta - T0*logf(R0/R_NTC) );
	present_temperature_C = present_temperature_K-Kelvin;

	// apply thermal protection and update hardware error register
	float const max_temperature_C = regs[REG_TEMPERATURE_LIMIT];
	if(present_temperature_C>max_temperature_C)
	{
		// set overheating error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_OVERHEATING;
		//HAL_Serial_Print(&serial,"h");
	}
	else
	{
		// clear overheating error
		regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_OVERHEATING);
	}
}

// low level function
// this function update present_voltage_V
// this function update REG_HARDWARE_ERROR_STATUS register (set/reset HW_ERROR_BIT_VOLTAGE bit)
// this function use REG_LOW_VOLTAGE_LIMIT and REG_HIGH_VOLTAGE_LIMIT registers
// the voltage ADC samples are collected with phase current samples
void LL_FOC_Update_Voltage()
{
	// process input voltage (STM32G431-ESC1 specific)
	{
		static float const R68 = 169.0f; // kohm
		static float const R76 = 18.0f; // kohm
		static float const alpha_voltage = 0.05f;
		present_voltage_V = (vbus_input_adc/4096.0f*3.3f*(R68+R76)/R76)*alpha_voltage+(1.0f-alpha_voltage)*present_voltage_V;
	}

	// apply voltage protection and update
	float const min_voltage_V = regs[REG_LOW_VOLTAGE_LIMIT];
	float const max_voltage_V = regs[REG_HIGH_VOLTAGE_LIMIT];
	if((present_voltage_V>max_voltage_V)||(present_voltage_V<min_voltage_V))
	{
		// set voltage error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_VOLTAGE;
		//HAL_Serial_Print(&serial,"v");
	}
	else
	{
		// clear voltage error
		regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_VOLTAGE);
	}
}


// user API function
// this function synchronize physical and electrical angles, set motor normal/reverse rotation, and check pole pairs
// this function uses REG_MOTOR_POLE_PAIRS register
int API_FOC_Calibrate()
{
    // reset setpoints
    setpoint_electrical_angle_rad = 0.0f;
    setpoint_flux_voltage_V = 0.0f;

	// reset settings
	regs[REG_INV_PHASE_MOTOR] = 0;
	regs[REG_MOTOR_SYNCHRO_L] = 0;
	regs[REG_MOTOR_SYNCHRO_H] = 0;
	regs[REG_MOTOR_SYNCHRO_H] = 0;

	// change mode
	regs[REG_CONTROL_MODE] = REG_CONTROL_MODE_POSITION_FLUX;

	// find natural direction

	// set electrical angle
	setpoint_electrical_angle_rad = M_3PI_2;
	setpoint_flux_voltage_V = 1.0f; // hard-coded V setpoint
	HAL_Delay(100);

    // move one electrical revolution forward
    for (int i = 0; i <=500; ++i )
    {
    	setpoint_electrical_angle_rad = M_3PI_2 + M_2PI * i / 500.0f;
    	HAL_Delay(2);
    }
    HAL_Delay(200);
    // take and angle in the middle
    positionSensor_update();
    float const mid_angle = positionSensor_getRadians();

    // move one electrical revolution backward
    for (int i = 500; i >=0; --i )
    {
    	setpoint_electrical_angle_rad = M_3PI_2 + M_2PI * i / 500.0f;
    	HAL_Delay(2);
    }
    HAL_Delay(200);
    // take and angle in the end
    positionSensor_update();
    float const end_angle = positionSensor_getRadians();

    // release motor
    setpoint_electrical_angle_rad = 0.0f;
    setpoint_flux_voltage_V = 0.0f;

	// change mode
	regs[REG_CONTROL_MODE] = REG_CONTROL_MODE_IDLE;

    // determine the direction the sensor moved
    float const delta_angle = mid_angle-end_angle;
    if(fabsf(delta_angle)<0.1f) // arbitrary delta angle
    {
    	HAL_Serial_Print(&serial,"Calibration failed (motor did not turn)\n",0 );
    	return 1; // failed calibration
    }
    if(delta_angle>0.0f)
    {
    	regs[REG_INV_PHASE_MOTOR] = 0;
    	HAL_Serial_Print(&serial,"Normal (%d)\n",0 ); // CCW
    }
    else
    {
    	regs[REG_INV_PHASE_MOTOR] = 1;
    	HAL_Serial_Print(&serial,"Reverse (%d)\n",1 ); // CW
    }

    // check pole pairs
    float const reg_pole_pairs = regs[REG_MOTOR_POLE_PAIRS];
    if( fabsf(fabsf(delta_angle)*reg_pole_pairs-M_2PI) > 0.5f )
    {
    	HAL_Serial_Print(&serial,"PP error (%d)\n",(int)( M_2PI/fabsf(delta_angle) ) );
    	return 2; // failed calibration
    }

    // set electrical angle
    setpoint_electrical_angle_rad = 0.0f;
    setpoint_flux_voltage_V = 1.0f; // hard-coded V setpoint

	// change mode
	regs[REG_CONTROL_MODE] = REG_CONTROL_MODE_POSITION_FLUX;

	// wait
    HAL_Delay(1000);
    positionSensor_update();
    float const reverse = regs[REG_INV_PHASE_MOTOR] == 0 ? 1.0f : -1.0f;
    float const phase_synchro_offset_rad = normalize_angle(-positionSensor_getRadians()*reg_pole_pairs*reverse);
	HAL_Serial_Print(&serial,"Synchro (%d)\n",(int)(RADIANS_TO_DEGREES(phase_synchro_offset_rad)) );
	regs[REG_MOTOR_SYNCHRO_L] = LOW_BYTE((int)RADIANS_TO_DEGREES(phase_synchro_offset_rad));
	regs[REG_MOTOR_SYNCHRO_H] = HIGH_BYTE((int)RADIANS_TO_DEGREES(phase_synchro_offset_rad));

    // release motor
    setpoint_electrical_angle_rad = 0.0f;
    setpoint_flux_voltage_V = 0.0f;

	// change mode
	regs[REG_CONTROL_MODE] = REG_CONTROL_MODE_IDLE;

	// store calibration into EEPROM
	store_eeprom_regs();

	return 0; // calibration success
}


void API_FOC_Service_Update()
{
	// check temperature and voltage
	LL_FOC_Update_Temperature();
	LL_FOC_Update_Voltage();
}

// user API function
// this function process an closed-loop FOC from flux and torque current setpoints
// this function allow on-the-go synchronization angle adjustment
// the open loop mode means that the present Id and Iq are forced to 0
//    this may require adjustment of the Kp and Ki of both flux and torque PI regulator
void API_FOC_Torque_Update()
{
// TODO CACHE REGISTER, DO NOT ACCESS REGISTER FROM HERE
// TODO CACHE REGISTER, DO NOT ACCESS REGISTER FROM HERE
// TODO CACHE REGISTER, DO NOT ACCESS REGISTER FROM HERE
// TODO CACHE REGISTER, DO NOT ACCESS REGISTER FROM HERE

	// performance monitoring
	uint16_t t_begin = __HAL_TIM_GET_COUNTER(&htim6);

	float Vd = 0.0f;
	float Vq = 0.0f;
	float cosine_theta = 0.0f;
	float sine_theta = 1.0f;

	// check control mode
	switch(regs[REG_CONTROL_MODE])
	{
	case REG_CONTROL_MODE_IDLE:
		{
			pid_reset(&flux_pi);
			pid_reset(&torque_pi);
		}
		break;
	case REG_CONTROL_MODE_POSITION_VELOCITY_TORQUE:
		{
			// process absolute position, and compute theta ahead using average processing time and velocity
			float const absolute_position_rad = positionSensor_getRadiansEstimation(t_begin);

			// process theta for Park and Clarke Transformation and compute cosine(theta) and sine(theta)
			float const phase_offset_rad = DEGREES_TO_RADIANS((int16_t)(MAKE_SHORT(regs[REG_MOTOR_SYNCHRO_L],regs[REG_MOTOR_SYNCHRO_H])));
			float const reg_pole_pairs = regs[REG_MOTOR_POLE_PAIRS];
			float const reverse = regs[REG_INV_PHASE_MOTOR] == 0 ? 1.0f : -1.0f;
			float const phase_synchro_offset_rad = DEGREES_TO_RADIANS((float)(MAKE_SHORT(regs[REG_GOAL_SYNCHRO_OFFSET_L],regs[REG_GOAL_SYNCHRO_OFFSET_H]))); // manual synchro triming

			// after this line ~11µs

			float const theta_rad = fmodf(absolute_position_rad*reg_pole_pairs*reverse,M_2PI) + phase_offset_rad + phase_synchro_offset_rad; // theta

			// after this line ~10µs
			API_CORDIC_Processor_Update(theta_rad,&cosine_theta,&sine_theta);


			// phase current (Ia,Ib,Ic) [0..xxxmA] to (Ialpha,Ibeta) [0..xxxmA] [Clarke Transformation]
			float const present_Ialpha = (2.0f*motor_current_mA[0]-motor_current_mA[1]-motor_current_mA[2])/3.0f;
			float const present_Ibeta = INV_SQRT3*(motor_current_mA[1]-motor_current_mA[2]);
			// Note : Ialpha synchonized with Ia (same phase/sign)
			// Note : Ibeta follow Iaplha with 90° offset

			// after this line ~8µs

			// (Ialpha,Ibeta) [0..xxxmA] to (Id,Iq) [0..xxxmA] [Park Transformation]
			present_Id_mA =  present_Ialpha * cosine_theta + present_Ibeta * sine_theta;
			present_Iq_mA = -present_Ialpha * sine_theta   + present_Ibeta * cosine_theta;

			// compute Vd and Vq
			// computation ~4µs


			// compute Id and Iq errors
			float const error_Id = setpoint_flux_current_mA   - present_Id_mA;
			float const error_Iq = setpoint_torque_current_mA - present_Iq_mA;
			// flux PI
			float const flux_Kp = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_FLUX_CURRENT_KP_L],regs[REG_PID_FLUX_CURRENT_KP_H])))/100000.0f;
			float const flux_Ki = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_FLUX_CURRENT_KI_L],regs[REG_PID_FLUX_CURRENT_KI_H])))/100000000.0f;
			Vd = pi_process_antiwindup_clamp(
					&flux_pi,
					error_Id,
					flux_Kp,
					flux_Ki,
					present_voltage_V // output_limit
			);
			// torque PIFF
			float const torque_Kp = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_TORQUE_CURRENT_KP_L],regs[REG_PID_TORQUE_CURRENT_KP_H])))/100000.0f;
			float const torque_Ki = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_TORQUE_CURRENT_KI_L],regs[REG_PID_TORQUE_CURRENT_KI_H])))/100000000.0f;
			Vq = pi_process_antiwindup_clamp(
					&torque_pi,
					error_Iq,
					torque_Kp,
					torque_Ki,
					present_voltage_V // output_limit
			);

			// voltage norm saturation Umax = Udc/sqrt(3)
			// computation ~0.5µs
			float const Vmax = present_voltage_V*INV_SQRT3;
			float const Vnorm = sqrtf(Vd*Vd+Vq*Vq);
			if(Vnorm>Vmax)
			{
				float const k = fabsf(Vmax/Vnorm);
				Vq *= k;
				Vd *= k;
			}


		}
		break;
	case REG_CONTROL_MODE_POSITION_FLUX:
		{
			// don t use PI
			pid_reset(&flux_pi);
			pid_reset(&torque_pi);

			// compute theta
			float const theta_rad = normalize_angle(setpoint_electrical_angle_rad);

			// compute cosine and sine
			API_CORDIC_Processor_Update(theta_rad,&cosine_theta,&sine_theta);

			// compute (Vd,Vq)
			Vd = setpoint_flux_voltage_V; // torque setpoint open loop
			Vq = 0.0f; // no torque
		}
		break;
	}
	// do inverse clarke and park transformation and update 3-phase PWM generation
	LL_FOC_set_phase_voltage(Vd,Vq,cosine_theta,sine_theta,present_voltage_V);


	// performance monitoring
	uint16_t const t_end = __HAL_TIM_GET_COUNTER(&htim6);
	uint16_t const t_tp = t_end-t_begin;
	static const float alpha_performance_monitoring = 0.001f;
	average_processing_time_us = (1.0f-alpha_performance_monitoring)*average_processing_time_us+alpha_performance_monitoring*(float)t_tp;
	++foc_counter;
}


float API_FOC_Get_Present_Torque_Current()
{
	return present_Iq_mA;
}

float API_FOC_Get_Present_Flux_Current()
{
	return present_Id_mA;
}

float API_FOC_Get_Present_Voltage()
{
	return present_voltage_V;
}

float API_FOC_Get_Present_Temp()
{
	return present_temperature_C;
}

float API_FOC_Get_Processing_Time()
{
	return average_processing_time_us;
}

float API_FOC_Get_Processing_Frequency()
{
	return (float)foc_counter/(float)HAL_GetTick()*1000.0f;
}

void API_FOC_It(ADC_HandleTypeDef *hadc)
{
	if(hadc==&hadc1)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			// phase current
			motor_current_input_adc[0] = ADC1_DMA[1];
			motor_current_mA[0]= ((float)motor_current_input_adc[0]-motor_current_input_adc_offset[0])*motor_current_input_adc_KmA;
			++current_samples;
			// aux
			potentiometer_input_adc = ADC1_DMA[2];
			vbus_input_adc = ADC1_DMA[3];
			temperature_input_adc = ADC1_DMA[4];
		}
		else
		{
			motor_current_input_adc_offset[0] = ALPHA_CURRENT_SENSE_OFFSET*(float)(ADC1_DMA[1]) + (1.0f-ALPHA_CURRENT_SENSE_OFFSET)*motor_current_input_adc_offset[0];
		}
	}
	if(hadc==&hadc2)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			// phase current
			motor_current_input_adc[1] = ADC2_DMA[1];
			motor_current_input_adc[2] = ADC2_DMA[2];
			motor_current_mA[1]= ((float)motor_current_input_adc[1]-motor_current_input_adc_offset[1])*motor_current_input_adc_KmA;
			motor_current_mA[2]= ((float)motor_current_input_adc[2]-motor_current_input_adc_offset[2])*motor_current_input_adc_KmA;
			current_samples+=2;
		}
		else
		{
			motor_current_input_adc_offset[1] = ALPHA_CURRENT_SENSE_OFFSET*(float)(ADC2_DMA[1]) + (1.0f-ALPHA_CURRENT_SENSE_OFFSET)*motor_current_input_adc_offset[1];
			motor_current_input_adc_offset[2] = ALPHA_CURRENT_SENSE_OFFSET*(float)(ADC2_DMA[2]) + (1.0f-ALPHA_CURRENT_SENSE_OFFSET)*motor_current_input_adc_offset[2];
		}
	}

	if(current_samples>=3)
	{
		current_samples=0;
		// FOC loop
		API_FOC_Torque_Update();
	}
}

