/*
 * 006spi_cmd_handling.c
 *
 *      Author: Red Russian Army
 */

/*
 * 	PB14 ---> SPI2_MISO
 * 	PB15 ---> SPI2_MOSI
 * 	PB13 ---> SPI2_SCLK
 * 	PB12 ---> SPI2_NSS
 * 	ALT function mode : 5
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

//extern void initialise_monitor_handles();

// command codes
#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON			1
#define LED_OFF			0

// arduino analog pins
#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

// arduino led
#define LED_PIN 		9

void delay(void)
{
	//	this delay is ~200ms when system clock is 16MHz
	for(uint32_t i=0 ; i < 500000/2 ; ++i);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // HW slave management enabled for NSS

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIObutton;

	//	initialize structure variables with 0
	memset(&GPIObutton, 0, sizeof(GPIObutton));

	// BUTTON CONFIGURATION PA0
	GPIObutton.pGPIOx = GPIOA;
	GPIObutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIObutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIObutton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIObutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIObutton);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}

	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read = 0xff;

	//initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	// this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// this function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	printf("SPI initialized\n");

	/*
	 * making SSOE 1 does NSS outpu enable.
	 * The NSS pin is automatically managed by the hardware
	 * i.e when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		// wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button debouncing ~200ms delay
		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// 1. CMD_LED_CTRL <pin_no(1)>		<value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// send arguments
			SPI_SendData(SPI2, args, 2);
			printf("COMMAND_LED_CTRL executed\n");
		}
		// end of COMMAND_LED_CTRL

	// 2. CMD_SENSOR_READ 	<analog pin number(0)>

		// wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button debouncing ~200ms delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			// send arguments
			SPI_SendData(SPI2, args, 1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// delay for slave to be ready with data
			delay();

			// send some dummy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ %d\n", analog_read);
		}
	// end of the CMD_SENSOR_READ

	// 3. CMD_LED_READ 	< pin number(1)>

		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button debouncing ~200ms delay
		delay();

		commandcode = COMMAND_LED_READ;

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_VerifyResponse(ackbyte)) {
			args[0] = LED_PIN;

			// send arguments
			SPI_SendData(SPI2, args, 1);

			// do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// delay for slave to be ready with data
			delay();

			// send some dummy bits (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("COMMAND_LED_READ %d\n", led_status);
		}
	// end of the CMD_LED_READ

	// 4. CMD_PRINT 	<len(2)>  <message(len)>

		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			;

		// to avoid button debouncing ~200ms delay
		delay();

		commandcode = COMMAND_PRINT;

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t message[] = "Hello ! How are you?";
		if (SPI_VerifyResponse(ackbyte)) {
			args[0] = strlen((char*)message);

			// send arguments
			SPI_SendData(SPI2, args, 1);

			// send message
			SPI_SendData(SPI2, message, args[0]);
			printf("COMMAND_PRINT executed\n");
		}
	// end of the CMD_PRINT

	// 4. CMD_ID_READ

		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
			;

		// to avoid button debouncing ~200ms delay
		delay();

		commandcode = COMMAND_ID_READ;

		// send command
		SPI_SendData(SPI2, &commandcode, 1);

		// do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t id[10];
		uint8_t i = 0;
		if (SPI_VerifyResponse(ackbyte)) {
			// read 10 bytes id from the slave
			for(i=0 ; i<10 ; i++)
			{
				// send dummy byte to fetch data from slave
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[11] = '\0';

			printf("COMMAND_ID : %s \n", id);
		}
	// end of the CMD_ID_READ

		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY) );

		// disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI Communication Closed\n");
	}

	return 0;
}

