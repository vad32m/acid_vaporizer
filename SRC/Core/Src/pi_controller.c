/*
 * pi_controller.c
 *
 *  Created on: Apr 9, 2021
 *      Author: vadym.mishchuk
 */

#include <pi_controller.h>

static PI_params globalParams;

static int32_t integrator;

void PI_init(PI_params* params)
{
	globalParams = *params;
	integrator = 0;
}

void PI_reset(void)
{
	integrator = 0;
}

uint16_t PI_process(int16_t current)
{
	int16_t currError = globalParams.desired - current;
	integrator += currError;

	if (integrator > globalParams.integratorMax)
	{
		integrator = globalParams.integratorMax;
	}
	else if (integrator < globalParams.integratorMin)
	{
		integrator = globalParams.integratorMin;
	}

	int32_t output = currError * globalParams.proportionalCoef + integrator * globalParams.integratorCoef;

	if (output > 0)
	{
		return (uint16_t) output;
	}
	else
	{
		return 0;
	}
}
