/*
 * PI_Regulator.hpp
 *
 *  Created on: 29.01.2019
 *      Author: Piotr Klimkowski
 */

#ifndef PID_H_
#define PID_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct
	{
		float p;
		float i;
		float d;
		float last_error;
		float sum_error;
		float dt;
		float setpoint;
		float input;
		float dead_zone;
		float last_value;
		float lower_limit;
		float upper_limit;
		float sum_clamping_coefficient;
		uint32_t previuos_time;
		uint8_t dt_autoupdate;

	} PIDStruct;

	void PID_init(PIDStruct *pid);
	void PID_reset(PIDStruct *pid);
	float PID_calc(PIDStruct *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
