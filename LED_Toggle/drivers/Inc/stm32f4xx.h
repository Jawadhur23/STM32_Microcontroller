/*
 * stm32f4xx.h
 *
 *  Created on: Sep 16, 2023
 *      Author: ADMIN
 *      This is an Driver Header File Inc-> Application Header File
 *      							  Src->Source Header File
 *      This Header File should be included in the Path while Building the Project
 *      C/C++ Build ->MCU GCC Compiler -> include path
 */


#include<stdio.h>

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

/*
 * base address of Flash and SRAM Memories
 */
#define FLASH_BASEADDR			0x00000000U 	/*Initial value of Base Address U denotes Unsigned*/
#define SRAM_BASEADDR			0x20000000U		/*SRAM Base Address for Storing the Data in Microcontroller*/
												/*SRAM is 112KB after this SAM2 Occur which is of 64KB we can convert 112*1024 change to Hex Value	*/
#define SRAM					SRAM_BASEADDR
#define SRAM2_BASEADDR			020001C000
#define ROM  					0x1FFF0000 		/*System Memory is an ROM*/

#define _vo volatile
/*
 *AHBx and APBx Bus Peripheral base Addresses
 *AHBx which is needed for high Speed data Communications
 *APBx is used for peripherals which speed Communication would Suffice
 */
#define PERIPH_BASE					0x40000000		/* From this address onwards peripheral Register Start*/
#define APB1PERIPH_BASEADDR			0x40000000		/* From this address onwards Base Address of Peripheral APB1 bus Start*/
#define APB2PERIPH_BASEADDR			0x40010000		/* From this address onwards Base Address of Peripheral APB2 bus Start*/
#define AHB1PERIPH_BASEADDR			0x40020000		/* From this address onwards Base Address of Peripheral AHB1 bus Start*/
#define AHB2PERIPH_BASEADDR			0x50000000		/* From this address onwards Base Address of Peripheral AHB2 bus Start*/

/*
 * This Address are from STM32F407DISCOVERY Board
 * Base Addresses of Peripherals which are hanging on AHB1 bus
 */
#define GPIOABASEADDR				0x40020000+0x0000		/*we can also mention 0x4002+0x0000 */
#define GPIOBBASEADDR				0x40020000+0x0400
#define GPIOCBASEADDR				0x40020000+0x0800
#define GPIODBASEADDR				0x40020000+0x0C00
#define GPIOEBASEADDR				0x40020000+0x1000
#define GPIOFBASEADDR				0x40020000+0x1400
#define GPIOGBASEADDR				0x40020000+0x1800
#define GPIOHBASEADDR				0x40020000+0x1C00
#define GPIOIBASEADDR				0x40020000+0x2000
#define GPIOJBASEADDR				0x40020000+0x3000
#define GPIOKBASEADDR				0x40020000+0x2800
#define RCCBASEADDR					0x40020000+0x3800

/*
 * This Address are from STM32F407DISCOVERY Board
 * Base Address of Peripherals which are hanging on APB1 bus
 */
#define I2C1BASEADDR				0x40000000+0x5400
#define I2C2BASEADDR				0x40000000+0x5800
#define SPI2BASEADDR				0x40000000+0x3800
#define SPI3BASEADDR				0x40000000+0x3C00
#define USART2BASEADDR				0x40000000+0x4400
#define USART3BASEADDR				0x40000000+0x4800
#define UART4BASEADDR				0x40000000+0x4C00
#define UART5BASEADDR				0x40000000+0x5000

/*
 * This Address are from STM32F407DISCOVERY Board
 * Base Address of Peripherals which are hanging on APB2 bus
 */
#define SPI1						0x40010000+0x3400
#define USART1						0x40010000+0x1000
#define USART6						0x40010000+0x1400
#define EXTI						0x40010000+0x3C00
#define SYSCFG						0x40010000+0x3800

/*Peripheral Register definitions structures
 *
 *
 * Note:Register of a Peripheral are specific to MCU
 * Number of Register of SPI Peripheral of STM32F4xx family of MCUs may be different
 * In this we have to check the correct RM Manual for a Particular MCU
 */
typedef struct
{
	_vo uint32_t MODER;				/* Address offset 0x00*/
	_vo uint32_t OTYPER;			/* Address offset 0x04*/
	_vo uint32_t OSPEEDR;			/* Address offset 0x08*/
	_vo uint32_t PUPDR;				/* Address offset 0x0C*/
	_vo uint32_t IDR;				/* Address offset 0x10*/
	_vo uint32_t ODR;				/* Address offset 0x14*/
	_vo uint32_t BSRR;				/* Address offset 0x18*/
	_vo uint32_t LCKR;				/* Address offset 0x1C*/
	_vo uint32_t AFR[2];			/* Address offset 0x1C*//* GPIO alternate function low register, AF[0]; GPIO Alternate function high register, AF[1]*/
}GPIO_RegDef_t;						/*GPIO_RegDef_t, which represents the register map of a GPIO.  GPIO peripheral's control registers*/


/*RCC Register definitions structure
 *
 */
typedef struct
{
 _vo uint32_t CR;						/* Address offset 0x00 */
 _vo uint32_t PLLCFGR;					/* Address offset 0x04 */
 _vo uint32_t CFGR;						/* Address offset 0x08 */
 _vo uint32_t CIR;						/* Address offset 0x0C */
 _vo uint32_t AHB1RSTR;					/* Address offset 0x10 */
 _vo uint32_t AHB2RSTR;					/* Address offset 0x14 */
 _vo uint32_t AHB3RSTR;					/* Address offset 0x18 */
  uint32_t RESERVED0;					/* Address offset 0x1C */
 _vo uint32_t APB1RSTR;					/* Address offset 0x20 */
 _vo uint32_t APB2RSTR;					/* Address offset 0x24 */
  uint32_t RESERVED1;					/* Address offset 0x28 */
  uint32_t RESERVED2;					/* Address offset 0x2C */
 _vo uint32_t AHB1ENR;					/* Address offset 0x30 */
 _vo uint32_t AHB2ENR;					/* Address offset 0x34 */
 _vo uint32_t AHB3ENR;					/* Address offset 0x38 */
  uint32_t RESERVED3;					/* Address offset 0x3C */
 _vo uint32_t APB1ENR;					/* Address offset 0x40 */
 _vo uint32_t APB2ENR;					/* Address offset 0x44 */
  uint32_t RESERVED4;					/* Address offset 0x48 */
  uint32_t RESERVED5;					/* Address offset 0x4C */
 _vo uint32_t AHB1LPENR;				/* Address offset 0x50 */
 _vo uint32_t AHB2LPENR;				/* Address offset 0x54 */
 _vo uint32_t AHB3LPENR;				/* Address offset 0x58 */
  uint32_t RESERVED6;					/* Address offset 0x5C */
 _vo uint32_t APB1LPENR;				/* Address offset 0x60 */
 _vo uint32_t APB2LPENR;				/* Address offset 0x64 */
  uint32_t RESERVED7;					/* Address offset 0x68 */
  uint32_t RESERVED8;					/* Address offset 0x6C */
 _vo uint32_t BDCR;						/* Address offset 0x70 */
 _vo uint32_t CSR;						/* Address offset 0x74 */
  uint32_t RESERVED9;					/* Address offset 0x78 */
  uint32_t RESERVED10;					/* Address offset 0x7C */
 _vo uint32_t SSCGR;					/* Address offset 0x80 */
 _vo uint32_t PLLI2SCFGR;				/* Address offset 0x84 */
 _vo uint32_t PLLSAICFGR;				/* Address offset 0x88*/
 _vo uint32_t DCKCFCFGR;				/* Address offset 0x8X */
}RCC_RegDef_t;

/*
 *Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
//GPIO_RegDef_t*pGPIOA = GPIOA
//Structure represents GPIO Register Map -> we are using pointer to point the GPIOA Will be having Structure. from here we can dereference and access the functions in structures
#define GPIOA 						((GPIO_RegDef_t*)GPIOABASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOB						((GPIO_RegDef_t*)GPIOBBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOC						((GPIO_RegDef_t*)GPIOCBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOD						((GPIO_RegDef_t*)GPIODBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOE						((GPIO_RegDef_t*)GPIOEBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOF						((GPIO_RegDef_t*)GPIOFBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOG						((GPIO_RegDef_t*)GPIOGBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOH						((GPIO_RegDef_t*)GPIOHBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOI						((GPIO_RegDef_t*)GPIOIBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOJ						((GPIO_RegDef_t*)GPIOJBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define GPIOK						((GPIO_RegDef_t*)GPIOKBASEADDR)/*This expression casts (converts) the memory address GPIOABASEADDR to a pointer of type GPIO_RegDef_t*  */
#define RCC							((RCC_RegDef_t*)RCCBASEADDR)/*This expression casts (converts) the memory address RCCBASEADDR to a pointer of type RCC_RegDef_t*  */

/*
 * Clock will be used in the program so we are enabling Now!!!
 *
 *Clock Enable Macros for GPIOx peripherals
 */
//#define GPIOA_PERI_CLOCK_ENABLE() //Briefly we will write
#define GPIOA_PCLK_EN()			RCC->AHB1ENR |= ( 1<<0 )
#define GPIOB_PCLK_EN()			RCC->AHB1ENR |= ( 1<<1 )
#define GPIOC_PCLK_EN()			RCC->AHB1ENR |= ( 1<<2 )
#define GPIOD_PCLK_EN()			RCC->AHB1ENR |= ( 1<<3 )
#define GPIOE_PCLK_EN()			RCC->AHB1ENR |= ( 1<<4 )
#define GPIOF_PCLK_EN()			RCC->AHB1ENR |= ( 1<<5 )
#define GPIOG_PCLK_EN()			RCC->AHB1ENR |= ( 1<<6 )
#define GPIOH_PCLK_EN()			RCC->AHB1ENR |= ( 1<<7 )
#define GPIOI_PCLK_EN()			RCC->AHB1ENR |= ( 1<<8 )
#define GPIOJ_PCLK_EN()			RCC->AHB1ENR |= ( 1<<9 )
#define GPIOK_PCLK_EN()			RCC->AHB1ENR |= ( 1<<10)

/**
 *
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1<<23))

/**
 *
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1<<15))



/**
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN			(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN			(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN			(RCC->APB1ENR |= (1<<18))
#define USART4_PCLK_EN			(RCC->APB1ENR |= (1<<19))
#define USART5_PCLK_EN			(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN1			(RCC->APB2ENR |= (1<<5)))

/**
 * Clock Enable Macros for SYSCFGx Peripherals
 */
#define SYSCFG_PCLK_EN			(RCC->APB2ENR |= (1<<14))
/*
 * Clock will be used in the program so we are enabling Now!!!
 *
 *Clock Disable Macros for GPIOx peripherals
 */
//#define GPIOA_PERI_CLOCK_ENABLE() //Briefly we will write
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<8))
#define GPIOJ_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<9))
#define GPIOK_PCLK_DI()			(RCC->AHB1ENR &= ~( 1<<10))
/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1<<21))

/**
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI_PCLK_DI()			(RCC->APB1ENR &= ~(1<<21))


/**
 * Clock Disable Macros for USARTx Peripherals
 */

/**
 *
 * Clock Disable Macros for SYSCFGx Peripherals
 */

/*
 *Macros to Reset GPIOx Peripherals
 */
#define GPIOA_REG_RESET()		do{(RCC->AHB1RSTR  |= (1<<0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOJ_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<9)); (RCC->AHB1RSTR &= ~(1 << 9)); }while(0)//Here we should not put ; this because when we call the function we actually use there
#define GPIOK_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<10)); (RCC->AHB1RSTR &= ~(1 << 10)); }while(0)//Here we should not put ; this because when we call the function we actually use there


/*
 * Genric Macros which can be used for GPIO Peripherals and Many..
 */
#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#include "stm32f4xx_gpio_driver.h"


#endif /* INC_STM32F4XX_H_ */

