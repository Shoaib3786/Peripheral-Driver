/*
 * 001_ToggleLED_PushPull.c
 *
 *  Created on: Jun 28, 2024
 *      Author: shoaib
 */

/*
* This .C file is for toggling on-board LED lights, but the this toggling must be commanded using
	API's to communicate user given commands to the controller thorugh GPIO Driver written for it.
*/


#include "stm32f411xx_gpio_drivers.h"  // this also include mcu specific header file which is in it already included
#include "stm32f411xx.h"
#include <stdio.h>
#include <string.h>


/*
 * creating software delay;
 * */
void delay(){

	for (uint32_t i=0; i<500000; i++);
}


int main(){


	GPIO_handle_t gpio_handle_OrangeLED, gpio_handle_GreenLED, gpio_handle_RedLED, gpio_handle_BlueLED, gpioButton;

	memset(&gpioButton,0,sizeof(gpioButton));  //this initializes zero to all the members of structure to avoid garbage value
	memset(&gpio_handle_GreenLED, 0, sizeof(gpio_handle_GreenLED));  //this initializes zero to all the members of structure to avoid garbage value


	//orange LED
	gpio_handle_OrangeLED.pGPIOx_baseAddr = GPIOD;

	gpio_handle_OrangeLED.GPIO_config.GPIO_mode = GPIO_MODE_OUT;
	gpio_handle_OrangeLED.GPIO_config.GPIO_pin_number = GPIO_PIN_13;
	gpio_handle_OrangeLED.GPIO_config.GPIO_output_type = GPIO_TYPE_PUSH_PULL;
	gpio_handle_OrangeLED.GPIO_config.GPIO_pullup_pulldown = GPIO_PUPD_NOPULLUP_PULLDOWN;
	gpio_handle_OrangeLED.GPIO_config.GPIO_speed = GPIO_SPEED_HIGH;


	//Green LED
	gpio_handle_GreenLED.pGPIOx_baseAddr = GPIOD;

	gpio_handle_GreenLED.GPIO_config.GPIO_mode = GPIO_MODE_OUT;
	gpio_handle_GreenLED.GPIO_config.GPIO_pin_number = GPIO_PIN_12;
	gpio_handle_GreenLED.GPIO_config.GPIO_output_type = GPIO_TYPE_PUSH_PULL;
	gpio_handle_GreenLED.GPIO_config.GPIO_pullup_pulldown = GPIO_PUPD_NOPULLUP_PULLDOWN;
	gpio_handle_GreenLED.GPIO_config.GPIO_speed = GPIO_SPEED_HIGH;


	//Red LED
	gpio_handle_RedLED.pGPIOx_baseAddr = GPIOD;

	gpio_handle_RedLED.GPIO_config.GPIO_mode = GPIO_MODE_OUT;
	gpio_handle_RedLED.GPIO_config.GPIO_pin_number = GPIO_PIN_14;
	gpio_handle_RedLED.GPIO_config.GPIO_output_type = GPIO_TYPE_PUSH_PULL;
	gpio_handle_RedLED.GPIO_config.GPIO_pullup_pulldown = GPIO_PUPD_NOPULLUP_PULLDOWN;
	gpio_handle_RedLED.GPIO_config.GPIO_speed = GPIO_SPEED_HIGH;


	//Blue LED
	gpio_handle_BlueLED.pGPIOx_baseAddr = GPIOD;

	gpio_handle_BlueLED.GPIO_config.GPIO_mode = GPIO_MODE_OUT;
	gpio_handle_BlueLED.GPIO_config.GPIO_pin_number = GPIO_PIN_15;
	gpio_handle_BlueLED.GPIO_config.GPIO_output_type = GPIO_TYPE_PUSH_PULL;
	gpio_handle_BlueLED.GPIO_config.GPIO_pullup_pulldown = GPIO_PUPD_NOPULLUP_PULLDOWN;
	gpio_handle_BlueLED.GPIO_config.GPIO_speed = GPIO_SPEED_HIGH;


	//Button (NORMAL)
//	gpioButton.pGPIOx_baseAddr = GPIOA;
//
//	gpioButton.GPIO_config.GPIO_mode = GPIO_MODE_IN;
//	gpioButton.GPIO_config.GPIO_pin_number = GPIO_PIN_0;
////	gpio_handle_BlueLED.GPIO_config.GPIO_output_type = GPIO_TYPE_PUSH_PULL;
//	gpioButton.GPIO_config.GPIO_pullup_pulldown = GPIO_PUPD_NOPULLUP_PULLDOWN;
//	gpioButton.GPIO_config.GPIO_speed = GPIO_SPEED_HIGH;
//

	//Button (for INTERRUPT)
	gpioButton.pGPIOx_baseAddr = GPIOA;

	gpioButton.GPIO_config.GPIO_mode = GPIO_MODE_IT_RT;    //The moe is neither OUTPUT nor INPUT instead it will be either RISING/Failing or both trigger if to get the Interrupt
	gpioButton.GPIO_config.GPIO_pin_number = GPIO_PIN_0;
	gpioButton.GPIO_config.GPIO_pullup_pulldown = GPIO_PUPD_NOPULLUP_PULLDOWN;
	gpioButton.GPIO_config.GPIO_speed = GPIO_SPEED_HIGH;


	// RCC_clock enable:
	GPIO_PeriCLKControl(GPIOD, ENABLE);  //CLK ENABLE PORT D

	GPIO_PeriCLKControl(GPIOA, ENABLE);	 //CLK ENABLE PORT A



	//INTIALIZE GPIOs;(to initial all the GPIO config registers)
	GPIO_init(&gpio_handle_OrangeLED);

	GPIO_init(&gpio_handle_GreenLED);

	GPIO_init(&gpio_handle_RedLED);

	GPIO_init(&gpio_handle_BlueLED);

	GPIO_init(&gpioButton);


	//For button we have at pin-0 at port-A; so for interrupt we choose to EXTI line 0.
	GPIO_IRQConfig(IRQ_NO_EXTI0, NVIC_PRI_NO_EXTI0, ENABLE);



	uint8_t count=0;





	//infinite loop
	while (1){

		count = count + 1;
		printf("my counter: %d", count);


//		status = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0);
//		printf("Status from input pin: %d\n", status);
//
//		if (status){
//			delay();
//			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
////			delay();
//			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_13);
////			delay();
//			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_14);
////			delay();
//			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_15);
////			delay();
//		}


	}


	return 0;

}


// So, when the button pressed interrupt rises, then this handler run and blink LED
// symboling INTERRUPT rise.

void EXTI0_IRQHandler(void){

	delay();//due to debouncing button led is not bliking properly so introduce delay

	GPIO_IRQHandling(GPIO_PIN_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);

}



