/*
 * load_control.h
 *
 *  Created on: Apr 4, 2021
 *      Author: vadym.mishchuk
 */

#ifndef INC_LOAD_CONTROL_H_
#define INC_LOAD_CONTROL_H_

#include <stdint.h>

void LOAD_CONTROL_init(void);

void LOAD_CONTROL_setLoad(uint8_t loadPercent);

#endif /* INC_LOAD_CONTROL_H_ */
