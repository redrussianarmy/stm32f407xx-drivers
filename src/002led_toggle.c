/*
 * 002led_toggle.c
 *
 *	LED TOGGLING WITH OPEN DRAIN CONFIGURATION
 *
 *      Author: Red Russian Army
 */

#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i=0 ; i < 500000 ; ++i);
}

int main(void)
{
	GPIO_Handle_t GPIOled;

	GPIOled.pGPIOx = GPIOD;
	GPIOled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	// because of internal pull up resistor has high value (40kohm), led will blink very weak

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GPIOled);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
