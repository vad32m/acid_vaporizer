/*
 * display.h
 *
 *  Created on: Mar 14, 2021
 *      Author: vadym.mishchuk
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include <stdint.h>

void DISPLAY_Init(void);

void DISPLAY_SetNumber(uint16_t number);

void DISPLAY_StartBlinking(void);

void DISPLAY_StopBlinking(void);

#endif /* INC_DISPLAY_H_ */
