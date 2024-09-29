/*
 * stm32f411xx_gpio_drivers.h
 *
 *  Created on: Jun 23, 2024
 *      Author: shoaib
 */

#ifndef INC_STM32F411XX_GPIO_DRIVERS_H_
#define INC_STM32F411XX_GPIO_DRIVERS_H_

#include "stm32f411xx.h"


// GPIOS configuration registers, which will be edited by the user at application level for their customization
typedef struct{

	uint8_t GPIO_pin_number;			// Possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_mode;					// Possible values from @GPIO_MODES
	uint8_t GPIO_speed;					// Possible values from @GPIO_SPEED
	uint8_t GPIO_output_type;			// Possible values from @GPIO_TYPE
	uint8_t GPIO_pullup_pulldown;		// Possible values from @GPIO_PUPD
	uint8_t GPIO_alt_function_mode;

}GPIO_config_t;


// This structure contains pointers to access GPIOS and its configuration registers
typedef struct{

	GPIO_Reg_t *pGPIOx_baseAddr;		// pointer pointing to the base address of GPIO peripheral
	GPIO_config_t GPIO_config;  		// for holding configuration members

}GPIO_handle_t;


/*
 * @GPIO_PIN_NUMBERS
 *
 * GPIO PIN NUMBERS(0 - 15)
 * STM32 has total 16 GPIO pins, that's why only 0-15 possible pin values
 * */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15


/*
 * @GPIO_MODES
 *
 * GPIOx MODER REGISTER DIFFERENT CONFIG
 *  -> Like(GPIOs input, output, alternate Function, Analog)
 * */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_AF		2
#define GPIO_MODE_ANLG		3

#define GPIO_MODE_IT_RT		4	//IT-> Input, RT-> Rising Edge Trigger (for interrupt detection as it also avaliable in INPUT MODE)
#define	GPIO_MODE_IT_FT		5	//IT-> Input, fT-> Falling Edge Trigger (for interrupt detection as it also avaliable in INPUT MODE)
#define GPIO_MODE_IT_RFT	6	//IT-> Input, RfT-> Rising and falling Edge Trigger (for interrupt detection as it also avaliable in INPUT MODE)



/*
 * @GPIO_SPEED
 *
 * GPIOx OUTPUT SPEED REGISTER(OSPEEDR) DIFFERENT CONFIG
 *  -> Like(GPIOs LOW, MEDIUM, FAST HIGH)
 * */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3



/*
 * @GPIO_TYPE
 *
 * GPIOX OUTPUT TYPE REGISTER(OTYPER) DIFFERENT CONFIG
 * */
#define GPIO_TYPE_PUSH_PULL			0
#define GPIO_TYPE_OPEN_DRAIN		1



/*
 * @GPIO_PUPD
 *
 * GPIOx PULLUP/PULLDOWN REGISTER(PUPDR) DIFFERENT CONFIG
 *  -> Like(GPIOs LOW, MEDIUM, FAST HIGH)
 * */
#define GPIO_PUPD_NOPULLUP_PULLDOWN			0
#define GPIO_PUPD_PULLUP					1
#define GPIO_PUPD_PULLDOWN					2
#define GPIO_PUPD_RESERVED					3



/***********************************************************************************************************
 * 											APIs supported by Drivers
 * Below functions drivers has, so that API could provide these functionalities to the User Application level
 ***********************************************************************************************************/


/*
 * GPIO Initialize/Deinitialize:
 *  */
void GPIO_init(GPIO_handle_t *pGPIOHandle); 	//GPIO Initialization function, will take care of intializing the GPIOs pin,port,mode...and other configuration
void GPIO_deInit(GPIO_Reg_t *pGPIOx_baseAddr);	//GPIO DeInitialization function -> will reset all the configuration registers(MODER, IDR...) of GPIOx -> to do so, instead of individually reseting all configuration, we can RESET it in one shot through RCC RESET REGISTER(RCC_AHB1RSTR) by SETTING which ever GPIOx pin you want to RESET.


/*
 * GPIO Peripheral Clock Control
 *  */
void GPIO_PeriCLKControl(GPIO_Reg_t *pGPIOx_baseAddr, uint8_t EnorDi);


/*
 * GPIO Read & Write
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_t *pGPIOx_baseAddr, uint8_t PinNumber);	//return value will be 0 or 1
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_t *pGPIOx_baseAddr);  					// return is uint16 because port has 16 pins
void GPIO_WriteToOutputPin(GPIO_Reg_t *pGPIOx_baseAddr, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_Reg_t *pGPIOx_baseAddr, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx_baseAddr, uint8_t PinNumber);



/*
 * GPIO Interrup Handler
 *  */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);		//Configuring the IRQ number for the interrupts.
void GPIO_IRQHandling(uint8_t PinNumber);	// If any interrupt occurs from the pin, then API will call this function to execute the interrupt process for that pin



#endif /* INC_STM32F411XX_GPIO_DRIVERS_H_ */



