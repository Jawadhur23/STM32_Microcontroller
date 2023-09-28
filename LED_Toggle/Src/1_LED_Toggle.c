/*
 * 1_LED_Toggle.c
 *
 *  Created on: 18-Sep-2023
 *      Author: ADMIN
 */
// Using Push Pull Configuration

#include "stm32f4xx.h"

void delay(void)
{
	for( uint32_t i=0; i<500000; i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);
	while(1)
	{
		GPIO_ToggleOutput(GPIOD,GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
