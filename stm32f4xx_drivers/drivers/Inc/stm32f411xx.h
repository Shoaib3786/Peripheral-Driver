/*
 * stm32f411xx.h
 *
 *  Created on: Jun 22, 2024
 *      Author: shoaib
 */


#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_



/* **************************************
 * PROCESSOR SPECIFIC ADDRESSES
 **************************************** */

//***** NVIC REGISTERS ADDRESS *****

#define NVIC_ISER0	( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1	( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2	( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3	( (__vo uint32_t*)0xE000E10c )


#define NVIC_ICER0	( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1	( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2	( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3	( (__vo uint32_t*)0XE000E18c )



//***** NVIC PRIORITY REGISTERS ADDRESS *****

#define NVIC_PR_BASE_ADDR	( (__vo uint32_t*)0xE000E400 )


// MCU specific priority bits in IPR register
#define NO_OF_BIT_IMPLEMENTED	4



/* MEMORY BASE ADDRESSES */
#define FLASH_BASEADDR 	0x08000000U   	/*Consider the base address of Flash Memory, U->unsigned*/
#define	SRAM_BASEADDR	0x20000000U   	/*Consider the base address of SRAM, U->unsigned*/
#define	ROM_BASEADDR	0x1FFF0000U		/*Consider the base address of system memory, U->unsigned*/


/* BUSSES BASES ADDRESSES */
#define APB1_BASEADDR	0x40000000U		/*Base addresses of APB buses*/
#define APB2_BASEADDR	0x40010000U		/*Base addresses of APB buses*/
#define	AHB1_BASEADDR	0x40020000U		/*Base addresses of Fast AHB buses*/
#define AHB2_BASEADDR	0x50000000U		/*Base addresses of Fast AHB buses*/


#define RCC_BASEADDR	(AHB1_BASEADDR + 0x3800)


/* ALL PERIPHERALS BASE ADDRESS
 *	AHB1 -> GPIOA, GPIOB...,GPIE
 *	APB1 -> I2C1, I2C2,I2C3, SPI1, SPI2, USART2
 * 	APB2 -> SPI3, USART1, USART6, EXTI, SYSCFG*/

#define GPIOA_BASEADDR	AHB1_BASEADDR
#define	GPIOB_BASEADDR	(AHB1_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR	(AHB1_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR	(AHB1_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR	(AHB1_BASEADDR + 0x1000U)
#define	GPIOH_BASEADDR	(AHB1_BASEADDR + 0x1C00U)

#define I2C1_BASEADDR	(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR	(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR	(APB1_BASEADDR + 0x5C00)
#define	SPI2_BASEADDR	(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR	(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR	(APB1_BASEADDR + 0x4400)

#define SPI1_BASEADDR	(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR	(APB2_BASEADDR + 0x3400)
#define SPI5_BASEADDR	(APB2_BASEADDR + 0x5000)
#define	USART1_BASEADDR	(APB2_BASEADDR + 0x1000)
#define	USART6_BASEADDR	(APB2_BASEADDR + 0x1400)
#define EXTI_BASEADDR	(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR	(APB2_BASEADDR + 0x3800)



#define __vo volatile  // use volatile for peripheral address to avoid any issues during optimization
#include<stdint.h>


/****** STRUCTURE METHOD:  FOR MAKING THE PERIPHERAL REGISTERS ADDRESSES EFFICIENTLY ******/
typedef struct{

	__vo uint32_t MODER;	// offset 0x00, these offset is being auto-maticallly understood, because size of uint32_t is 4byte only
	__vo uint32_t OTYPER;	// offset 0x04
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];	//AFR[0] -> for AFRL, AFR[1]-> for AFRH

}GPIO_Reg_t;


#define GPIOA 		((GPIO_Reg_t*) GPIOA_BASEADDR)  //GPIOA_BASEADDR becomes the pointer to the GPIO_Reg_t holding its base address, now this can be accessed by GPIOA
#define GPIOB 		((GPIO_Reg_t*) GPIOB_BASEADDR)
#define GPIOC 		((GPIO_Reg_t*) GPIOC_BASEADDR)
#define GPIOD 		((GPIO_Reg_t*) GPIOD_BASEADDR)
#define GPIOE 		((GPIO_Reg_t*) GPIOE_BASEADDR)
#define GPIOH 		((GPIO_Reg_t*) GPIOH_BASEADDR)

#define RCC			((RCC_Reg*) RCC_BASEADDR)
#define EXTI		((EXTI_Reg_t*) EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_reg_t*) SYSCFG_BASEADDR)


/****** STRUCTURE METHOD:  FOR MAKING THE EXTI (Interrupt) CONTROLLER REGISTERS ******/
typedef struct{

	__vo uint32_t IMR;		// Interrupt mask register
	__vo uint32_t EMR;		// Event mask register
	__vo uint32_t RTSR;		// Rising trigger selection register
	__vo uint32_t FTSR;		// Falling trigger selection register
	__vo uint32_t SWIER;	// Software interrupt event register
	__vo uint32_t PR;  		// Pending register

}EXTI_Reg_t;


typedef struct{

	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;

}SYSCFG_reg_t;




/***** STURCTURE METHOD: FOR MAKING RCC CLOCK REGISTERS ******/

typedef struct{

	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t reserved0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t reserved1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t reserved2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t reserved3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t reserved4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t reserved5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t reserved7;
	uint32_t DCKCFGR;

}RCC_Reg;

//RCC_Reg *ptr_rcc = (RCC_Reg*)RCC;

//RCC->AHB1ENR = 0x20;



/**** RCC CLOCK ENABLING/DISABLING ****/

/****** ENABLE CLOCK FOR GPIO PERIPHERALS ******/
#define GPIOA_CLK_EN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_CLK_EN()	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_CLK_EN()	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_CLK_EN()	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_CLK_EN()	(RCC->AHB1ENR |= (1<<4))
#define GPIOH_CLK_EN()	(RCC->AHB1ENR |= (1<<7))

/****** ENABLE CLOCK FOR SYSCFG  ******/
#define SYSCFG_CLK_EN()	(RCC->APB2ENR) |= (1<<14)


/****** ENABLE CLOCK FOR I2C PERIPHERALS ******/
#define I2C1_CLK_EN()	(RCC->APB1ENR) |= (1<<21)
#define I2C2_CLK_EN()	(RCC->APB1ENR) |= (1<<22)
#define I2C3_CLK_EN()	(RCC->APB1ENR) |= (1<<23)


/****** ENABLE CLOCK FOR SPI PERIPHERALS ******/
#define SPI1_CLK_EN()	(RCC->APB2ENR) |= (1<<12)
#define SPI2_CLK_EN()	(RCC->APB1ENR) |= (1<<14)
#define SPI3_CLK_EN()	(RCC->APB1ENR) |= (1<<15)
#define SPI4_CLK_EN()	(RCC->APB2ENR) |= (1<<13)
#define SPI5_CLK_EN()	(RCC->APB2ENR) |= (1<<20)


/****** ENABLE CLOCK FOR USART PERIPHERALS ******/
#define USART1_CLK_EN()	(RCC->APB2ENR) |= (1<<4)
#define USART2_CLK_EN()	(RCC->APB1ENR) |= (1<<17)
#define USART6_CLK_EN()	(RCC->APB2ENR) |= (1<<5)


//DISABLING THE CLOCK

/****** DISABLE CLOCK FOR GPIO PERIPHERALS ******/
#define GPIOA_CLK_DI()	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_CLK_DI()	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_CLK_DI()	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_CLK_DI()	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_CLK_DI()	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_CLK_DI()	(RCC->AHB1ENR &= ~(1<<7))


/****** DISABLE CLOCK FOR I2C PERIPHERALS ******/
#define I2C1_CLK_DI()	(RCC->APB1ENR) &= ~(1<<21)
#define I2C2_CLK_DI()	(RCC->APB1ENR) &= ~(1<<22)
#define I2C3_CLK_DI()	(RCC->APB1ENR) &= ~(1<<23)


/****** DISABLE CLOCK FOR SPI PERIPHERALS ******/
#define SPI1_CLK_DI()	(RCC->APB2ENR) &= ~(1<<12)
#define SPI2_CLK_DI()	(RCC->APB1ENR) &= ~(1<<14)
#define SPI3_CLK_DI()	(RCC->APB1ENR) &= ~(1<<15)
#define SPI4_CLK_DI()	(RCC->APB2ENR) &= ~(1<<13)
#define SPI5_CLK_DI()	(RCC->APB2ENR) &= ~(1<<20)


/****** DISABLE CLOCK FOR USART PERIPHERALS ******/
#define USART1_CLK_DI()	(RCC->APB2ENR) &= ~(1<<4)
#define USART2_CLK_DI()	(RCC->APB1ENR) &= ~(1<<17)
#define USART6_CLK_DI()	(RCC->APB2ENR) &= ~(1<<5)


/****** IRQ NUMBERS on EXTIx (Through Vector table) ******/
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/****** PRIORITY NUMBERS on EXTIx (Through Vector table) ******/
#define NVIC_PRI_NO_EXTI0			13
#define NVIC_PRI_NO_EXTI1			14
#define NVIC_PRI_NO_EXTI2			15
#define NVIC_PRI_NO_EXTI3			16
#define	NVIC_PRI_NO_EXTI4			17
#define NVIC_PRI_NO_EXTI9_5			30
#define NVIC_PRI_NO_EXTI10_15		47



/****** DISABLE CLOCK FOR SYSCFG  ******/
#define SYSCFG_CLK_DI()	(RCC->APB2ENR) &= ~(1<<14)


/****** this macro return code between(0 to 7) for the given base address ******/
//USING CONDITION STATMENT in MACROS:
//(condition) ? True_value : False
//(condition) ? True_value : \ ->(\ means elseif)

#define GPIO_BASEADDR_TO_CODE(x)	   ((x==GPIOA) ? 0 :\
									    (x==GPIOB) ? 1 :\
										(x==GPIOC) ? 2 :\
										(x==GPIOD) ? 3 :\
										(x==GPIOE) ? 5 :\
										(x==GPIOH) ? 6 :0);




/*
 *  PERIPHERAL RESET REGISTER OF RCC_AHB1RSTR)
 *
 *  USE OF THIS RESET REGISTER?
 *  	- We use this reset register to De-Initialize the pins. RESETING/CLEARING all the changes made during Initializing
 *
 *  This is used to reset the all pins of a particular GPIO port
 *  - to reset first set the GPIO port as 1 in RCC_AHB1RSTR register than back to 0
 *
 *  MORE THAN 1 STATEMENT IN MACRO FUNCTION?
 *  	- USE DO WHILE loop --> do {statement_1; statement2} while{0}
 *  	- after while{0} no need to give termination (;) because when u will call this MACRO function there eventually at the end u will use the termination(;)
 *
 * */
#define GPIOA_RCC_RESET() 			do{ (RCC->AHB1RSTR) |= (1<<0);   (RCC->AHB1RSTR) &= ~(1<<0);} while (0)
#define GPIOB_RCC_RESET() 			do{ (RCC->AHB1RSTR) |= (1<<1);   (RCC->AHB1RSTR) &= ~(1<<1);} while (0)
#define GPIOC_RCC_RESET() 			do{ (RCC->AHB1RSTR) |= (1<<2);   (RCC->AHB1RSTR) &= ~(1<<2);} while (0)
#define GPIOD_RCC_RESET() 			do{ (RCC->AHB1RSTR) |= (1<<3);   (RCC->AHB1RSTR) &= ~(1<<3);} while (0)
#define GPIOE_RCC_RESET() 			do{ (RCC->AHB1RSTR) |= (1<<4);   (RCC->AHB1RSTR) &= ~(1<<4);} while (0)
#define GPIOH_RCC_RESET() 			do{ (RCC->AHB1RSTR) |= (1<<7);   (RCC->AHB1RSTR) &= ~(1<<7);} while (0)







/**************************
 * GENERAL PURPOSE MACROS
 * (These MACROS may be used by other peripherals(I2C, GPIOs..)
 *  that's why defined here in MCU specific Header not GPIO_driver header )
 *************************/
#define ENABLE 				0
#define DISABLE 			1
#define SET					ENABLE
#define	RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET




#endif /* INC_STM32F411XX_H_ */













