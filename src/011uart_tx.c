/*
 * 011uart_tx.c
 *
 *      Author: Red Russian Army
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart2_handle;

void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USARTConfig.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USARTConfig.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USARTConfig.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USARTConfig.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USARTConfig.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USARTConfig.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void 	USART2_GPIOInit(void)
{
	GPIO_Handle_t USART_GPIOs;

	USART_GPIOs.pGPIOx = GPIOA;
	USART_GPIOs.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART_GPIOs.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART_GPIOs.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USART_GPIOs.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	USART_GPIOs.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART2 TX
	USART_GPIOs.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_2;
	GPIO_Init(&USART_GPIOs);

	//USART2 RX
	USART_GPIOs.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&USART_GPIOs);


}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is button GPIO configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led GPIO configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_ButtonInit();

	USART2_GPIOInit();

    USART2_Init();

    USART_PeripheralControl(USART2,ENABLE);

    while(1)
    {
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		USART_SendData(&usart2_handle,(uint8_t*)msg,strlen(msg));

    }

	return 0;
}
