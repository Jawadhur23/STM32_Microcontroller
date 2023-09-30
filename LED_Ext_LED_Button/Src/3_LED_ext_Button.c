/*
 * 1_LED_Toggle.c
 *
 *  Created on: 18-Sep-2023
 *      Author: ADMIN
 */
// Using Push Pull Configuration

#include "stm32f4xx.h"
#define HIGH 			1
#define BTN_PRESSED		HIGH
#define LOW				0
#define BTNS_PRESSED	LOW


void delay(void)
{
	for( uint32_t i=0; i<500000; i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed,GPIOBtn;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx = GPIOB; //GPIO is connected to PA0 so, we should write like this.
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_12;//GPIO is connected to PA0 so, we should write like this.
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;//We are using internal Pull up

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GPIOBtn);
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) == BTNS_PRESSED)
		{
			delay();//When you remove delay Button debounce will happen
			GPIO_ToggleOutput(GPIOD,GPIO_PIN_NO_8);//we cannot use GPIOPA14,15,13,4,3 ->as a GPIO_PIN_NO since these are debug Pins which is used for Debug

		}

	}
	return 0;
}
