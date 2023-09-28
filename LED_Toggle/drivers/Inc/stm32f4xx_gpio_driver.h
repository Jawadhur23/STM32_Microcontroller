/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: 17-Sep-2023
 *      Author: ADMIN
 */

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_

//MCU Header File
#include "stm32f4xx.h"

/*
 * This is a Handle Structure for a GPIO Pin
 */

typedef struct
{
	uint8_t		GPIO_PinNumber;				/*Possible Values from @GPIO_PIN_MODES*/
	uint8_t		GPIO_PinMode;				/*Possible Values from @GPIO_PIN_MODES*/
	uint8_t 	GPIO_PinSpeed;				/*Possible Values from @GPIO_PIN_SPEED*/
	uint8_t		GPIO_PinPuPdControl;		/*Possible Values from @GPIO_PIN_PullUpPullDown*/
	uint8_t		GPIO_PinOPType;				/*Possible Values from @GPIO_PIN_OutputType*/
	uint8_t		GPIO_PinAltFunMode;			/*Possible Values from @GPIO_PIN_AlternateFunctionMode*/

}GPIO_PinConfig_t;
typedef struct
{
	GPIO_RegDef_t *pGPIOx;					/* This Pin holds the base Address of the GPIO Port to which the pin Belongs */
	GPIO_PinConfig_t GPIO_PinConfig;		/*	This holds GPIO pin Configuration settings*/
}GPIO_Handle_t;

/*
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2 			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9 			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12 			12
#define GPIO_PIN_NO_13 			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15 			15

/*
 * GPIO Pin Possible MOdes
 */
// Non Interrupt Modes
//If value less than 3 Non Interrupt Modes
#define GPIO_MODE_IP	 		0			/*Interrupt Mode when pin is in  Interrupt Mode we can COnfigure Input Mode*/
#define GPIO_MODE_OP			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
//Interrupt Modes
//If Value Greater than 3 Interrupt Values
#define GPIO_MODE_IT_FT			4			/*Falling Edge Trigger*/
#define GPIO_MODE_IT_RT			5			/*Rising Edge Trigger*/
#define GPIO_MODE_IT_RFT		6			/*Rising Falling Edge Trigger*/

/*
 *GPIO Pin possible output Types
 */
#define GPIO_OP_TYPE_PP			0			/*OP->Output and PP->Push Pull*/
#define GPIO_OP_TYPE_OD			1			/*OD-> Output Drain*/

/*
 * GPIO PIn POssible Output Speeds
 */
#define GPIO_SPEED_LOW			0			/*Low Speed*/
#define GPIO_SPEED_MEDIUM		1			/*Medium Speed*/
#define GPIO_SPEED_FAST			2			/*High Speed*/
#define GPIO_SPEED_HIGH			3			/*Very High Speed*/


/*
 * GPIO Pin Pull up and Pull Down Configurations
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				0
#define GPIO_NO_PD				0

/*
 * APIs Supported by this driver
 * For more Information about the APIs check the function Definitions
 */

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi); 		/*by using this we will enable or Disable the peripheral Clock*/
/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);//Her we are Resetting the Value so, Just we need to make GPIO Peripherals Complement

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromOutputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);// Value ->PINSET or PINRESET
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutput(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
