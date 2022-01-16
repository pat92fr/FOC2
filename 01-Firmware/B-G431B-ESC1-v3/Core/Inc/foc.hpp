/*
 * foc.hpp
 *
 *  Created on: 16 janv. 2022
 *      Author: Patrick
 */

#ifndef INC_FOC_HPP_
#define INC_FOC_HPP_

struct foc
{
	foc();

	void calibrate();

	void enable();
	void disable();
	void update();

	void set_currents(float torque_ma, float flux ma);

	uint32_t get_timestamp_ms();
	uint16_t get_timestamp_us();

	float get_torque_current();
	float get_flux_current();
	float get_dc_voltage();
	float get_temperature();

	float get_processing_time();
	float get_processing_frequency();
};



#endif /* INC_FOC_HPP_ */
