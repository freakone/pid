/*
 * PI_Regulator.cpp
 *
 *  Created on: 29.01.2019
 *      Author: Piotr Klimkowski
 */
#include "pid.h"
#include "float.h"

#ifdef USE_HAL_DRIVER
#include "stm32f1xx_hal.h"
#endif

inline float PID_abs(float value)
{
	if (value < 0)
		value = value * -1.0f;
	return value;
}

void PID_init(PIDStruct *pid)
{
	pid->p = 0.0f;
	pid->i = 0.0f;
	pid->d = 0.0f;
	pid->last_error = 0.0f;
	pid->sum_error = 0.0f;
	pid->dt = 1.0f;
	pid->setpoint = 0.0f;
	pid->input = 0.0f;
	pid->dead_zone = 0.0f;
	pid->last_value = 0.0f;
	pid->lower_limit = -1.0f;
	pid->upper_limit = 1.0f;
	pid->sum_clamping_coefficient = 1.0f;
	pid->previuos_time = 0;
	pid->dt_autoupdate = 0;
}
void PID_reset(PIDStruct *pid)
{
	pid->last_value = 0;
	pid->last_error = 0;
	pid->sum_error = 0;
	pid->previuos_time = 0;
}

float PID_calc(PIDStruct *pid)
{
	float error = pid->setpoint - pid->input;

	if (PID_abs(error) <= pid->dead_zone && pid->last_value < pid->upper_limit && pid->last_value > pid->lower_limit)
	{
		return pid->last_value;
	}

#ifdef USE_HAL_DRIVER
	if (pid->dt_autoupdate)
	{

		const uint32_t current_tick = HAL_GetTick();
		if (pid->previuos_time == 0)
		{
			pid->dt = 1.0f;
		}
		else
		{
			pid->dt = (current_tick - pid->previuos_time) / 1000;

			if (current_tick < pid->previuos_time)
			{
				pid->dt = ((0xFFFFFFFF - pid->previuos_time) + current_tick) / 1000;
			}
		}

		pid->previuos_time = current_tick;
	}
#endif

	const float new_sum_error = pid->sum_error + error * pid->dt;
	const float derivative = (error - pid->last_error) / pid->dt;
	pid->last_value = pid->p * error + pid->i * new_sum_error + pid->d * derivative;
	pid->last_error = error;

	//anti windup for integral part
	if (pid->sum_clamping_coefficient < FLT_EPSILON ||
		(pid->last_value < pid->upper_limit * pid->sum_clamping_coefficient &&
		 pid->last_value > pid->lower_limit * pid->sum_clamping_coefficient))
	{
		pid->sum_error = new_sum_error;
	}

	if (pid->last_value > pid->upper_limit)
	{
		pid->last_value = pid->upper_limit;
	}
	else if (pid->last_value < pid->lower_limit)
	{
		pid->last_value = pid->lower_limit;
	}

	return pid->last_value;
}
