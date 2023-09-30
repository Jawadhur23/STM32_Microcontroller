/*
 * stm32f4xx_driver.c
 *
 *  Created on: 17-Sep-2023
 *      Author: ADMIN
 */
#include "stm32f4xx_gpio_driver.h"
#include "stm32f4xx.h"
/*
 * APIs Supported by this driver
 * For more Information about the APIs check the function Definitions
 */

/*
 * Peripheral Clock Setup
 */
/*************DOCUMENTATION SECTION**************************
 * @fn				- GPIO_PeriCLockControl
 *
 * @brief			- This function Enables or Disables Peripheral CLock for the given GPIO Port
 *
 * @param[in]		- base address of GPIO Peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in]		-
 *
 * @return 			- None
 *
 * @Note			- None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}

		else if(pGPIOx == GPIOD)
		{
					GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
				if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
				else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}
				else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}
				else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}
				else if(pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}
				else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}
				else if(pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}
				else if(pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}
				else if(pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}


}

/*
 * Init and De-Init
 */

/*************DOCUMENTATION SECTION**************************
 * @fn				- GPIO_Init
 * 					  GPIO_DeInit
 *
 * @brief			- Configure the Mode of GPIO Pin
 * 					  Configure the Speed
 * 					  Configure the Pull up and Pull down Register
 * 					  Configure the Output Type
 * 					  Configure the alternate Function
 *
 *					  We have written these structures in Header File about each COnfiguration
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return 			- None
 *
 * @Note			- None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t  temp=0;	//Temp Register
	//1. Configuring the Mode of GPIO Pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));//Since 2 set of bits combined we will Multiply with the Pin to get an Correct Pin
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |=temp;//Actual Register

		temp=0;
	}
	else
	{
		// In this whatever we are configuring is a peripheral specific
		//EXTI is a Peripheral Configuration
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)//IT Falling Edge
		{
			//1. Configure the FTSR(Falling Trigger Selection Register)
			//FTSR is a Register of EXTI
			EXTI -> FTSR |=(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the Corresponding RTSR bit
			EXTI -> RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.Configure the RTSR(Rising trigger Selection Register)
			EXTI -> RTSR |=(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the Corresponding FTSR bit
			EXTI -> FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//Configure both FTSR and RTSR
			//Configure FTSR bit
			EXTI -> RTSR |=(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Configure RTSR bit
			EXTI -> FTSR |=(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2.Configure the GPIO Port selection in SYSCFG_EXITCR
		uint8_t temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber %4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle -> pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG -> EXTICR[temp1]=portcode << (temp2 *4);

		//3.Enable the EXTI Interrupt delivery using IMR
		EXTI->IMR |= 1<<pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber;

	}
	temp=0;

	//2. Configure the Speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &=~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;

	temp=0;

	//3. COnfigure the PU->PullUp and PD->PullDown Register
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &=~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |=temp;

	temp=0;
	//4. COnfigure THE OPType
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx-> OTYPER |=temp;

	temp=0;

	//5. Configure the Alternate Functionality
	//This will be used when in PinMode is set to Alternate Functionality Mode or otherwise it wont be used.
	//In this Section 4 bits are dedicated as a Single Pin
	//Ex: Assume Temp1-> 6/8 = 0 when 0 it is AFR[0]
	//Again AFR |temp1}-> |6/8|=0 when it value << (4*temp2) i.e; pin is 6 from there 6*4=24th pi we will start to configure
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1, temp2;
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %8;
		pGPIOHandle->pGPIOx->AFR[temp1]= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1]= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)//Her we are Resetting the Value so, Just we need to make GPIO Peripherals Complement
//We should reset all the Registers of a Peripheral
{
			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}

			else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}
/*
 * Data Read and Write
 */

/*************DOCUMENTATION SECTION**************************
 * @fn				- GPIO_PeriCLockControl
 *
 * @brief			- This function Enables or Disables Peripheral CLock for the given GPIO Port
 *
 * @param[in]		- base address of GPIO Peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in]		-
 *
 * @return 			- None
 *
 * @Note			- None
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); //In the IDR Register we will Shift the PinNumber Mentioned to the Initial Position of 0x00000001

	return value;
}
uint16_t GPIO_ReadFromOutputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value=(uint16_t)(pGPIOx->IDR);//Since we need Entire Port
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)// Value ->PINSET or PINRESET
{
	//In this value can be written to value should be written to PinNumber
	if(Value == GPIO_PIN_SET)
	{
		//written 1 to the output data Register at the bit field Corresponding to the Pin Number
		pGPIOx-> ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx-> ODR |= ~(1<<PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutput(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx ->ODR = pGPIOx->ODR ^(1 << PinNumber);			//Here just we are Doing an bitwise XOR Operation for Toggling
}
/*
 * IRQ Configuration and ISR Handling
 */

/*************DOCUMENTATION SECTION**************************
 * @fn				- GPIO_IRQConfig
 *
 * @brief			- We will be Using ISER and ICER for enabling and disabling the interrupt
 *
 * @param[in]		- None
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in]		-
 *
 * @return 			- None
 *
 * @Note			- None
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	//In this whatever we are configuring is a Processor SPecific
	if(EnorDi == ENABLE)
	{
		if(IRQNumber<=31)
		{
			//Program ISER0 Register
			*NVIC_ISER0 |=(1<<IRQNumber);
		}
		else if(IRQNumber >=64 && IRQNumber<96)
		{
			//Program ISER1 Register which is from 32 to 63
			*NVIC_ISER1 |=(1<<IRQNumber % 32);
		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{
			//Program ISER2 Register which is from 64 to 95
			*NVIC_ISER3 |=(1<<IRQNumber %64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 Register
			*NVIC_ICER0 |=(1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{
			//Program ICER1 Register which ranges from 32 to 63
			*NVIC_ICER1 |=(1<<IRQNumber % 32);
		}
		else if(IRQNumber >=6 && IRQNumber <96)
		{
			//Program ICER2 Register which ranges from 6 to 95
			*NVIC_ICER3 |=(1<<IRQNumber %64);
		}
	}

}
/*
 * Interrupt Priority Register
 *
 * In ARM Cortex Mx Processor we will be having 60 Priority Registers - IPR0 -> IPR59
 * Each register has 32 bits - 4 Sections has been divided
 * Each Section of 1 IRQ
 * in program if we given with 236IRQ Number
 * we have to divide 236/4 since 4 section we will get 59 i.e, 59 IPR
 * we will find the section 236 % 4 i.e., =0 so we will program IRQ236_PRI
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1.First lets find out the IPR Register
	//Lower Priority bits are not implemented
	uint8_t iprx = IRQNumber /4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8 *iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber)
//GPIO Pin Interrupt Handling
//ISR are not implemented in Driver code, it is implemented in Application Layer user
//ISR does not take anything and does not write anything
{
	//Clear the EXTI pr Register corresponding to the pin number
	if(EXTI ->PR |= (1<<PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
