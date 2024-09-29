/*
 * stm32f411xx_gpio_drivers.c
 *
 *  Created on: Jun 23, 2024
 *      Author: shoaib
 */

#include "stm32f411xx_gpio_drivers.h"
#include<stdio.h>

/*
 * GPIO Initialize/De-initialize:
 *
 * -> GPIO Initialization function -> will take care of initializing the GPIOs pins(port,mode...)and other configuration
 *
 * -> GPIO DeInitialization function -> will reset all the configuration registers(MODER, IDR...)
 *  of GPIOx -> to do so, instead of individually reseting all configuration, we can RESET it in
 *  one shot through RCC RESET REGISTER(RCC_AHB1RSTR) by SETTING which ever GPIOx pin you want to
 *  RESET.
 *
 *  */


/* ***************************************************************************************
 * @fun				- GPIO_init
 *
 * @brief			- For initializing the structure values of GPIOx by USER INSERTED Values
 *
 * @param[in]		- base address of the GPIO port
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void GPIO_init(GPIO_handle_t *pGPIOHandle){

	uint32_t temp = 0;


	//TODO: 1.Initialize the GPIO MODER register
	if (pGPIOHandle->GPIO_config.GPIO_mode <= GPIO_MODE_ANLG){

		temp = (pGPIOHandle->GPIO_config.GPIO_mode << (2 * pGPIOHandle->GPIO_config.GPIO_pin_number) );  // 2* pin_number -> because moder register has 2 bits for each pins, therefore if I want to set pin 2 then left shift it 2*2 = 4th bit position of MODER register.
		pGPIOHandle->pGPIOx_baseAddr->MODER |= temp;  //use bitwise OR (|=) not assignment (=) ---> because if we use assignment(=) then it will affect other bit position, so to avoid that we should use bitwise OR.

	}


	//TODO: INTERRUPTS
	else{
		//for interrupt code (WHEN MODER AT INPUT STATE)
		// GPIO INTERRUPT PROCESSING=> GPIO send interrupt-> EXTI (Enable EXTI lines through IMR)-> NVIC (Enable IRQ number)
		//1. Enable interrupt delivery on EXTIx line (use IMR register) for user provided pin number (PIN SHOULD BE IN INPUT MODE).
		//2. Also Provide the Port number to the EXTI using SYSCFG_EXTICR.
		//3.




		//RISING TRIGGER (clear FAILING TRIGGER if any)
		if (pGPIOHandle->GPIO_config.GPIO_mode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= 1 << pGPIOHandle->GPIO_config.GPIO_pin_number; //SETTING
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_config.GPIO_pin_number); //CLEARING
		}

		//FALLING TRIGGER (clear RISING TRIGGER if any)
		else if(pGPIOHandle->GPIO_config.GPIO_mode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= 1 << pGPIOHandle->GPIO_config.GPIO_pin_number; //SETTING
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_config.GPIO_pin_number); //CLEARING
		}

		//RISING & FAILING TRIGGER
		else if(pGPIOHandle->GPIO_config.GPIO_mode == GPIO_MODE_IT_RFT){
			EXTI->RTSR |= 1 << pGPIOHandle->GPIO_config.GPIO_pin_number; //SETTING
			EXTI->FTSR |= 1 << pGPIOHandle->GPIO_config.GPIO_pin_number; //SETTING
		}


		//2. Configure GPIO Port selection on EXTIx line through SYSCFG_EXTICR.
		uint8_t temp1 = pGPIOHandle->GPIO_config.GPIO_pin_number/4;
		uint8_t temp2 = pGPIOHandle->GPIO_config.GPIO_pin_number%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx_baseAddr);
		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);



		//3. Enable EXTI interrupt Delivery using IMR(Interrupt Masking Register)
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_config.GPIO_pin_number;


	}


	//TODO:2. Input Mode -> Nothing to initialize for Input Mode, just directly implement the function.



	//TODO:3. initialize the OUTPUT TYPE REG
	temp = 0;
	temp = pGPIOHandle->GPIO_config.GPIO_output_type << pGPIOHandle->GPIO_config.GPIO_pin_number;
	pGPIOHandle->pGPIOx_baseAddr->OTYPER |= temp;



	//TODO:4. initialize the SPEED REG
	temp = 0;
	temp = pGPIOHandle->GPIO_config.GPIO_speed << (2 * pGPIOHandle->GPIO_config.GPIO_pin_number);  // 2* pin_number -> because speed register also has 2 bits for each pins, therefore if I want to set pin 2 then left shift it 2*2 = 4th bit position of SPEED register.
	pGPIOHandle->pGPIOx_baseAddr->OSPEEDR |= temp;



	//TODO:5. initialize the PULLUP/PULLDOWN REG
	temp = 0;
	temp = pGPIOHandle->GPIO_config.GPIO_pullup_pulldown << (2 * pGPIOHandle->GPIO_config.GPIO_pin_number);  // 2* pin_number -> because speed register also has 2 bits for each pins, therefore if I want to set pin 2 then left shift it 2*2 = 4th bit position of SPEED register.
	pGPIOHandle->pGPIOx_baseAddr->PUPDR |= temp;



	//TODO: 6. initialize the ALTERNATIVE FUNCTION REG
	if (pGPIOHandle->GPIO_config.GPIO_mode == GPIO_MODE_AF){
		//ALTERNATE FUNCTION CODE LATER
	}


}


/* ***************************************************************************************
 * @fun				- GPIO_deInit
 *
 * @brief			- For DeIntialzing the USER INSERTED Values in the GPIOx peripheral registers by RESETTING the changes through RCC_AHBI!RSTR register
 *
 * @param[in]		- base address of GPIO port
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void GPIO_deInit(GPIO_Reg_t *pGPIOx_baseAddr){

	if (pGPIOx_baseAddr == GPIOA){ // checking address containing in the pointer "pGPIOx_baseAddr"
		GPIOA_RCC_RESET();
	}
	else if(pGPIOx_baseAddr == GPIOB){
		GPIOB_RCC_RESET();
	}
	else if(pGPIOx_baseAddr == GPIOC){
		GPIOC_RCC_RESET();
	}
	else if(pGPIOx_baseAddr == GPIOD){
		GPIOD_RCC_RESET();
	}
	else if(pGPIOx_baseAddr == GPIOE){
		GPIOE_RCC_RESET();
	}
	else if(pGPIOx_baseAddr == GPIOH){
		GPIOH_RCC_RESET();
	}

}


/*
 * GPIO Peripheral Clock Control
 *  */

/* ***************************************************************************************
 * @fun				- GPIO_PeriCLKControl
 *
 * @brief			- This function is for enabling or disabling the peripheral clock
 *
 * @param[in]		- Base address of the GPIO port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_PeriCLKControl(GPIO_Reg_t *pGPIOx_baseAddr, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if (pGPIOx_baseAddr == GPIOA){ // checking address containing in the pointer "pGPIOx_baseAddr"
			GPIOA_CLK_EN();
		}
		else if(pGPIOx_baseAddr == GPIOB){
			GPIOB_CLK_EN();
		}
		else if(pGPIOx_baseAddr == GPIOC){
			GPIOC_CLK_EN();
		}
		else if(pGPIOx_baseAddr == GPIOD){
			GPIOD_CLK_EN();
		}
		else if(pGPIOx_baseAddr == GPIOE){
			GPIOE_CLK_EN();
		}
		else if(pGPIOx_baseAddr == GPIOH){
			GPIOH_CLK_EN();
		}

	}

	//Else disbale the clock
	else{
		if (pGPIOx_baseAddr == GPIOA){
			GPIOA_CLK_DI();
		}
		else if(pGPIOx_baseAddr == GPIOB){
			GPIOB_CLK_DI();
		}
		else if(pGPIOx_baseAddr == GPIOC){
			GPIOC_CLK_DI();
		}
		else if(pGPIOx_baseAddr == GPIOD){
			GPIOD_CLK_DI();
		}
		else if(pGPIOx_baseAddr == GPIOE){
			GPIOE_CLK_DI();
		}
		else if(pGPIOx_baseAddr == GPIOH){
			GPIOH_CLK_DI();
		}
	}

}


/*
 * GPIO Read & Write
 * */

/* ***************************************************************************************
 * @fun				- GPIO_ReadFromInputPin
 *
 * @brief			- This function is for reading from the Input pin
 *
 * @param[in]		- Base address of the GPIO port
 * @param[in]		- Specific Pin number from which you want to read
 *
 * @return			- returns (0 or 1) value
 *
 * @note			- none
 *
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_t *pGPIOx_baseAddr, uint8_t PinNumber){

	//Bring the input Bit position to the LSB and it with 1 to get its value, and return it, if value is 1 then that bit position is SET for input else NOT SET.
	uint8_t value = (pGPIOx_baseAddr->IDR >> PinNumber)& 1;
//	uint8_t value = (pGPIOx_baseAddr->IDR >> PinNumber);
	printf("Input reading value: %u\n", value);
	return value;


}


/* ***************************************************************************************
 * @fun				- GPIO_ReadFromInputPort
 *
 * @brief			-
 *
 * @param[in]		- Base address of GPIO port
 *
 * @return			- returns pin
 *
 * @note			- none
 *
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_Reg_t *pGPIOx_baseAddr){

	//Get the All the pins BIT position data of that port.
	uint16_t value = (pGPIOx_baseAddr->IDR);
	return value;
}


/* ***************************************************************************************
 * @fun				- GPIO_WriteToOutputPin
 *
 * @brief			-
 *
 * @param[in]		- Base address of GPIO port.
 * @param[in]		- Specific Pin number on to which you want to write.
 * @param[in]		- value (0 or 1)
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void GPIO_WriteToOutputPin(GPIO_Reg_t *pGPIOx_baseAddr, uint8_t PinNumber, uint8_t value){


	if(value == GPIO_PIN_SET){
		//THEN SET THE BIT POSITION IN THE ODR
		pGPIOx_baseAddr->ODR |= (1 << PinNumber);
	}
	else{
		// ELSE RESET/CLEAR THE BIT PSOTION IN THE ODR
		pGPIOx_baseAddr->ODR &= ~(1 << PinNumber);
	}

}


/* ***************************************************************************************
 * @fun				- GPIO_WriteToOutputPort
 *
 * @brief			- Writing the user value not in single pin, instead in all the pins of that Port
 *
 * @param[in]		- Base address of GPIO port.
 * @param[in]		- value (0 or 1)
 *
 * @return			- none
 *
 * @note			- none
 *
 * */

void GPIO_WriteToOutputPort(GPIO_Reg_t *pGPIOx_baseAddr, uint16_t value){

	pGPIOx_baseAddr->ODR = value;  // Just insert value into the whole pins of the corresponding GPIOx port
}


/* ***************************************************************************************
 * @fun				- GPIO_ToggleOutputPin
 *
 * @brief			- Toggle state can only happen if the previous state is different from next state otherwise it will remain same (to perform such logical operation it can be only done by XOR gate not other basic GATEs)
 *
 * @param[in]		- Base address of GPIO port.
 * @param[in]		- Specific Pin number
 *
 * @return			- none
 *
 * @note			- Toogling means changing the state of the Pin from it previous state
 *
 * */

void GPIO_ToggleOutputPin(GPIO_Reg_t *pGPIOx_baseAddr, uint8_t PinNumber){

	pGPIOx_baseAddr->ODR ^=(1<<PinNumber);
}


/*
 * GPIO Interrupt Handler:
 * */

/* ***************************************************************************************
 * @fun				- GPIO_IRQConfig
 *
 * @brief			- Configuring the IRQ number for the interrupts.
 *
 * @param[in]		- uint8_t IRQNumber
 * @param[in]		- uint8_t IRQPriority
 * @param[in]		- uint8_t EnorDi
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){


	//SETTING THE PRIORITY
	uint8_t iprx = IRQNumber / 4;  //we are finding which IPR register it is
	uint8_t iprx_section = IRQNumber % 4;

	uint32_t shiftamount = (8 * iprx_section) + (8 - NO_OF_BIT_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx *4) |= (IRQPriority << shiftamount); //multiplied by 8 because each section is 8 bit



	// ENABLE / DISABLE ISER AND ICER
	if (EnorDi == ENABLE){ //when Enable do ISER (Interrupt set enable register)

		if (IRQNumber <=31){
			//ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber >=32 && IRQNumber <=63){
			//ISER1 register
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32 );

		}else if(IRQNumber >=64 && IRQNumber <96){
			//ISER2 register
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64 );
		}

	}else{ //when DISABLE do ICER(Interrupt Clear enable register)
		if (IRQNumber <=31){
			//ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber >=32 && IRQNumber <=63){
			//ICER1 register
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32 );

		}else if(IRQNumber >=64 && IRQNumber <96){
			//ICER2 register
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64 );
		}
	}

}


/* ***************************************************************************************
 * @fun				- GPIO_IRQHandling
 *
 * @brief			-  This function is used, If any interrupt occurs from the pin, then API will
 * 					call this function to execute the interrupt process for that pin
 *
 * @param[in]		- Specific pin number
 *
 * @return			- none
 *
 * @note			- none
 *
 * */
void GPIO_IRQHandling(uint8_t PinNumber){

	//Clear PR register in the EXTI line
	if (EXTI->PR & (1 << PinNumber)){

		//clear the bit by setting 1 not 0 for PR(pending register) in EXTI
		EXTI->PR |= (1 << PinNumber);
	}

}








