/*
 * 4_Button_Interrupt.c
 *
 *  Created on: 29-Sep-2023
 *      Author: ADMIN
 */

/*
 * Connect an External Button to PD5 pin and toggle the LED Whenever interrupt is Triggered by the button Press
 *
 * Interrupt should be triggered during Falling edge of Button Press
 *
 */
#include <string.h>
#include "stm32f4xx.h"
#define HIGH 			1
#define BTN_PRESSED		HIGH
#define LOW				0
#define BTNS_PRESSED	LOW


void delay(void)
{
	//this will introduce 200ms delay when system clock is 16MHz
	for( uint32_t i=0; i<500000/2; i++);
}
int main(void)
{
	//In application we clear each and Every element to zero
	GPIO_Handle_t GpioLed, GPIOBtn;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

	GPIOBtn.pGPIOx = GPIOD; //GPIO is connected to PA0 so, we should write like this.
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;//GPIO is connected to PA0 so, we should write like this.
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_FAST;
	//GPIOBtn.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_OD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;//We are using internal Pull up

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GPIOBtn);

	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15); //Priority Configuration
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	//we have to go to startup file -> Vector Table
	void EXTI9_5_IRQHandler(void)
	{
		delay();//200ms
		GPIO_IRQHandling(GPIO_PIN_NO_5);
		GPIO_ToggleOutput(GPIOD, GPIO_PIN_NO_12);
	}
	return 0;
}
