/*
 * sensors.h
 *
 *  Created on: Mar 28, 2021
 *      Author: vadym.mishchuk
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <stdint.h>

typedef struct
{
	int16_t coldJunctionTemp;
	uint16_t thermocoupleTemp;
	uint16_t potentiometerAngle;
} SENSORS_Readings;

void SENSORS_init(void);

SENSORS_Readings SENSORS_getReadings(void);

#endif /* INC_SENSORS_H_ */
