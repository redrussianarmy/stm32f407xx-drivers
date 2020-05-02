/*
 * 003led_toggle_button.c
 *
 *	LED TOGGLING WITH PRESSING THE BUTTON
 *
 *      Author: Red Russian Army
 */

#define HIGH			1
#define BTN_PRESSED		HIGH

#include "stm32f407xx.h"

void delay(void)
{
	//	this delay is ~200ms when system clock is 16MHz
	for(uint32_t i=0 ; i < 500000/2 ; ++i);
}

int main(void)
{
	// BUTTON CONFIGURATION PA0
	GPIO_Handle_t GPIObutton;

	GPIObutton.pGPIOx = GPIOA;
	GPIObutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIObutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObutton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIObutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// LED CONFIGURATION PD12
	GPIO_Handle_t GPIOled;

	GPIOled.pGPIOx = GPIOD;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIObutton);
	GPIO_Init(&GPIOled);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, 0) == BTN_PRESSED)
		{
			delay();	//	to prevent button debouncing ~200ms
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}

	return 0;
}
