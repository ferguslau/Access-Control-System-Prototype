/*
 * debounce.txt
 *
 *  Created on: Mar 31, 2020
 *      Author: ShahEd Shahir
 */

#include <stdint.h>
#include "stm32f4xx_hal.h"


// Defining functions

void debounceInit()
{
	/*Configure GPIO pin */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}


void deBounceInit(uint16_t pin, char port, int8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0}; // Declaring GPIO_initStruct as GPIO_InitTypeDef Struct

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	/*Configure GPIO pin */

	switch (mode)
	{
	case 0: GPIO_InitStruct.Pull = GPIO_PULLUP; break;
	case 1: GPIO_InitStruct.Pull = GPIO_PULLDOWN; break;
	}

	switch (port)
	{
	case 'A': HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); break;
	case 'B': HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); break;
	case 'C': HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); break;
	}

}



int8_t deBounceReadPin(uint16_t pin, char port, int8_t mode)
{
	GPIO_PinState pinState = GPIO_PIN_RESET;
	int8_t pinStateWeAreLookingFor;
	int8_t stableInterval = 50;

	int32_t msTimeStamp = HAL_GetTick();

	/* let's do our first read of the pin
	 * and to do that we need to incorporate the port we're going to read*/
	switch (port)
	{
	case 'A': pinState = HAL_GPIO_ReadPin(GPIOA, pin); break;
	case 'B': pinState = HAL_GPIO_ReadPin(GPIOB, pin); break;
	case 'C': pinState = HAL_GPIO_ReadPin(GPIOC, pin); break;
	}

	/* is the GPIO pin high or low */
	if (pinState == GPIO_PIN_RESET) // if low, we're looking for more 0's
	{
		pinStateWeAreLookingFor = 0;
	}
	else
	{
		pinStateWeAreLookingFor = 1; //we're looking for more 1's
	}
	/* now, let's read the pin again until x stable ms have elapsed */
	while (HAL_GetTick()<(msTimeStamp + stableInterval))
	{
		switch (port)
		{
		case 'A': pinState = HAL_GPIO_ReadPin(GPIOA, pin); break;
		case 'B': pinState = HAL_GPIO_ReadPin(GPIOB, pin); break;
		case 'C': pinState = HAL_GPIO_ReadPin(GPIOC, pin); break;
		}

		if (pinState != pinStateWeAreLookingFor)
		{
			pinStateWeAreLookingFor = !pinStateWeAreLookingFor;
			msTimeStamp = HAL_GetTick();
		}
	}

	return (pinStateWeAreLookingFor);

}
