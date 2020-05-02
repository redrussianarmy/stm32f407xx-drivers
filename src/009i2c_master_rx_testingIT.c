/*
 * 009i2c_master_rx_testingIT.c
 *
 *      Author: Red Russian Army
 */

/*
 * PB6 -> SCL
 * PB7 -> SDA
 * ALT function mode: 4
 */

#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

// Flag variable
uint8_t rxCompleted = RESET;

#define MY_ADDR		0x61
#define SLAVE_ADDR	0x68

I2C_Handle_t I2C1Handle;

// data sending to slave
uint8_t receive_buf[32];

void delay(void)
{
	//	this delay is ~200ms when system clock is 16MHz
	for(uint32_t i=0 ; i < 500000/2 ; ++i);
}

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2CConfig.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2CConfig.I2C_DeviceAddress = MY_ADDR; // acc. to I2C spec outside of reserved addresses
	I2C1Handle.I2CConfig.I2C_FMDutyCycle = I2C_FM_DUTY_CYCLE_2; // it doesnt matter because we dont use fast mode
	I2C1Handle.I2CConfig.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIObutton;

	// BUTTON CONFIGURATION PA0
	GPIObutton.pGPIOx = GPIOA;
	GPIObutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIObutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	GPIObutton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIObutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIObutton);
}

int main(void)
{
	uint8_t commandcode;
	uint8_t len;

	GPIO_ButtonInit();

	// I2C1 pin inits
	I2C1_GPIOInits();

	// I2C1 peripheral configuration
	I2C1_Inits();

	// I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	while(1)
	{
		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button debouncing ~200ms delay
		delay();

		commandcode = 0x51;

		// according to "Master sending command to slave"
		while( I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY );

		while( I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY );

		commandcode = 0x52;
		while( I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY );

		while( I2C_MasterReceiveDataIT(&I2C1Handle, receive_buf, len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY );

		rxCompleted = RESET;

		// wait till RX is completed
		while( rxCompleted != SET )
		{

		}

		receive_buf[len+1] = '\0';

		rxCompleted = RESET;
	}
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		// printf("Tx is completed\n");
	}else if(AppEv == I2C_EV_RX_CMPLT)
	{
		// printf("Rx is completed\n");
		rxCompleted = SET;
	}else if(AppEv == I2C_ERROR_AF)
	{
		// printf("Error: ACK failure\n");
		// slave fails to send ACK for the byte sent from the master
		I2C_CloseSendData(pI2CHandle);

		// generate the stop condition to release the bus
		I2C_GenerateStopCondition(I2C1);

		// hang it infinite loop
		while(1);
	}else if(AppEv == I2C_EV_RX_CMPLT)
	{
		// printf("Tx is completed\n");
	}else if(AppEv == I2C_EV_RX_CMPLT)
	{
		// printf("Tx is completed\n");
	}
}
