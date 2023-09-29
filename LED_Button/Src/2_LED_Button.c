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


void delay(void)
{
	for( uint32_t i=0; i<500000; i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed,GPIOBtn;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx = GPIOA; //GPIO is connected to PA0 so, we should write like this.
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_0;//GPIO is connected to PA0 so, we should write like this.
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);

	GPIO_Init(&GPIOBtn);
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) == BTN_PRESSED)
		{
			GPIO_ToggleOutput(GPIOD,GPIO_PIN_NO_12);
			delay();//When you remove delay Button debounce will happen
		}

	}
	return 0;
}
