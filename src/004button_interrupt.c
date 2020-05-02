/*
 * 004button_interrupt.c
 *
 *	LED TOGGLING WITH BUTTON INTERRUPT
 *
 *      Author: Red Russian Army
 */

#include <string.h>		//	for memset function
#include "stm32f407xx.h"

void delay(void)
{
	//	this delay is ~200ms when system clock is 16MHz
	for(uint32_t i=0 ; i < 500000/2 ; ++i);
}

int main(void)
{

	GPIO_Handle_t GPIObutton, GPIOled;

	//	initialize structure variables with 0
	memset(&GPIObutton, 0, sizeof(GPIObutton));
	memset(&GPIOled, 0, sizeof(GPIOled));

	// BUTTON CONFIGURATION PA0
	GPIObutton.pGPIOx = GPIOA;
	GPIObutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIObutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIObutton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIObutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIObutton);

	// LED CONFIGURATION PD12

	GPIOled.pGPIOx = GPIOD;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOled);

	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET);

	//	IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI5);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();							//	to prevent button debouncing ~200ms

	GPIO_IRQHandling(GPIO_PIN_NO_0);	//	clear the pending event from EXTI line

	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
