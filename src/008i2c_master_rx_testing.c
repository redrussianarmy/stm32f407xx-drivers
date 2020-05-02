/*
 * 008i2c_master_rx_testing.c
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

	// enable the I2C peripheral

	while(1)
	{
		// wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		// to avoid button debouncing ~200ms delay
		delay();

		commandcode = 0x51;

		// according to "Master sending command to slave"
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, receive_buf, len, SLAVE_ADDR, I2C_DISABLE_SR);
	}
}
