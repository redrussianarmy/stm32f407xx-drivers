/*
 * 0010i2c_slave_tx_string.c
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

#define SLAVE_ADDR	0x68
#define MY_ADDR		SLAVE_ADDR

I2C_Handle_t I2C1Handle;

// data sending to slave
uint8_t TX_buf[32] = "STM32 Slave mode testing...";

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
	GPIO_ButtonInit();

	// I2C1 pin inits
	I2C1_GPIOInits();

	// I2C1 peripheral configuration
	I2C1_Inits();

	// I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	while(1);
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
	static uint8_t commandCode = 0;
	static uint8_t Cnt = 0;

	if(AppEv == I2C_EV_DATA_REQ)
	{
		// Master wants data and slave has to send it
		if(commandCode == 0x51)
		{
			// send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)TX_buf));
		}else if(commandCode == 0x52)
		{
			// send the contents of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx, TX_buf[Cnt++]);
		}
	}else if(AppEv == I2C_EV_DATA_RCV)
	{
		// data is waiting for the slave to read.
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}else if(AppEv == I2C_ERROR_AF)
	{
		// happens only druing slave transmitting
		// Master has sent the NACK
		// slave should understand that master doesnt need more data
		commandCode = 0xff;	//	invalidating
		Cnt = 0;
	}else if(AppEv == I2C_EV_STOP)
	{
		// happens only during slave reception
		// Master has ended the I2C communication with slave
	}
}
