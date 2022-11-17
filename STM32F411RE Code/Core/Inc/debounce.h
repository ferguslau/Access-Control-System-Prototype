/*
 * debounce.h
 *
 *  Created on: Nov 9, 2021
 *      Author: Fergus Lau
 */

#ifndef INC_DEBOUNCE_H_
#define INC_DEBOUNCE_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

void debounceInit();
void deBounceInit(uint16_t pin, char port, int8_t mode);
int8_t deBounceReadPin(uint16_t pin, char port, int8_t mode);

#endif /* INC_DEBOUNCE_H_ */
