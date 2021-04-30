/*
 * pi_controller.h
 *
 *  Created on: Apr 9, 2021
 *      Author: vadym.mishchuk
 */

#ifndef INC_PI_CONTROLLER_H_
#define INC_PI_CONTROLLER_H_

#include <stdint.h>

typedef struct
{
	float proportionalCoef;
	float integratorCoef;
	uint16_t integratorMax;
	int16_t integratorMin;
	uint16_t desired;
} PI_params;

void PI_init(PI_params* params);

void PI_reset(void);

uint16_t PI_process(int16_t current);

#endif /* INC_PI_CONTROLLER_H_ */
