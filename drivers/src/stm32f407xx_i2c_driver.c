/*
 * stm32f407xx_i2c_driver.c
 *
 *      Author: Red Russian Army
 */

#include "stm32f407xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

/***********************************************************************************************************
 *  @fn				-	I2C_GenerateStartCondition
 *
 *  @brief			-	This function generates start condition for a given I2C peripheral
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START );
}

/***********************************************************************************************************
 *  @fn				-	I2C_ExecuteAddressPhaseWrite
 *
 *  @brief			-	This function configures Master to WRITE
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *  @param[in]		-	Slave address
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // creating space for read&write bit
	SlaveAddr &= ~(1); 			// The Isb is R/nW bit which must be set to 0 for WRITE
	pI2Cx->DR = SlaveAddr;
}

/***********************************************************************************************************
 *  @fn				-	I2C_ExecuteAddressPhaseRead
 *
 *  @brief			-	This function configures Master to READ
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *  @param[in]		-	Slave address
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1; // creating space for read&write bit
	SlaveAddr |= 1; 			// The Isb is R/nW bit which must be set to 1 for READ
	pI2Cx->DR = SlaveAddr;
}

/***********************************************************************************************************
 *  @fn				-	I2C_ClearADDRFlag
 *
 *  @brief			-	This function clears ADDR flag
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;

	// check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// disable the ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// clear the ADDR flag (read SR1, read SR2)
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;	// to prevent "unused error"
			}
		}else
		{
			// clear the ADDR flag (read SR1, read SR2)
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void) dummyRead;	// to prevent "unused error"
		}
	}else{
		// device is in slave mode
		// clear the ADDR flag (read SR1, read SR2)
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void) dummyRead;	// to prevent "unused error"
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_GenerateStopCondition
 *
 *  @brief			-	This function generates STOP condition for a given I2C peripheral
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}

/***********************************************************************************************************
 *  @fn				-	I2C_SlaveEnableDisableCallbackEvents
 *
 *  @brief			-	This function enables or disables callback events
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}

/*
 *	Peripheral Clock Setup
 */
/***********************************************************************************************************
 *  @fn				-	I2C_PeriClockControl
 *
 *  @brief			-	This function enables or disables peripheral clock for a given I2Cx peripheral
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}


/*
 *	Init and De-init
 */
/***********************************************************************************************************
 *  @fn				-	I2C_Init
 *
 *  @brief			-	This function configures I2Cx Peripheral
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	// temporary register
	uint32_t tempreg = 0;

	// enable the clock for the I2Cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	if(pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		// first enable PE to enable ACK
		tempreg |= 1 << I2C_CR1_PE;
		pI2CHandle->pI2Cx->CR1 = tempreg;

		// ACK control bit
		tempreg |= pI2CHandle->I2CConfig.I2C_ACKControl << I2C_CR1_ACK;
		pI2CHandle->pI2Cx->CR1 = tempreg;
	}else{
		// no need to configure ACK=0
	}

	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= (RCC_GetPCLK1Value() / 1000000U) & 0x3F;	// convert the unit to MHz and mask
	pI2CHandle->pI2Cx->CR2 = tempreg;

	// program the device own address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2CConfig.I2C_DeviceAddress) << 1;	// shifted by 1 because 0th bit(ADD0) of I2C_OAR1 is not generally used (means 7 bit address of slave)
	tempreg |= (1 << 14);	// because of the sentence for 14th bit in I2C_OAR1 as "Should always be kept at 1 by software."
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF); // because CCR register is 16 bits. Other bits of tempreg must be masked
	}else
	{
		// mode is fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_CYCLE_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		}else if((pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_CYCLE_16_9))
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2CConfig.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF); // because CCR register is 16 bits. Other bits of tempreg must be masked
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// TRISE configuration
	if(pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		tempreg = (RCC_GetPCLK1Value() * I2C_TRISEMAX_SM) + 1;
	}else
	{
		// mode is fast mode
		tempreg = (RCC_GetPCLK1Value() * I2C_TRISEMAX_FM) + 1;
	}
	tempreg &= 0x3F;	// first 6 bits of TRISE reg are used
	pI2CHandle->pI2Cx->TRISE = tempreg;
}

/***********************************************************************************************************
 *  @fn				-	I2C_DeInit
 *
 *  @brief			-	This function resets the register of I2Cx Peripheral
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_GetFlagStatus
 *
 *  @brief			-	This function returns flag status for a given flag
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *  @param[in]		-	Flag name
 *
 *  @return			-	Flag status
 *
 *  @Note			-	none
 *
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***********************************************************************************************************
 *  @fn				-	I2C_PeripheralControl
 *
 *  @brief			-
 *
 *  @param[in]		-	base address of the I2Cx peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	enables or disables the I2Cx peripheral
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_MasterSendData
 *
 *  @brief			-	This function is used for sending data
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *  @param[in]		-	Slave address
 *  @param[in]		-	Repeated start condition
 *
 *  @return			-	none
 *
 *  @Note			-	This is blocking call
 *
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t Sr)
{
	// Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is completed by checking the SB flag in the SR1
	// until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// send the address of the slave with R/nW bit set to W(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddress);

	// confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// clear the ADDR flag according to its software sequence
	// until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	// send the data until Len becomes 0
	while (Len > 0)
	{
		while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) ); // wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	// when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// TXE=1, BTF=1 means that both SR and DR are empty and next transmission should begin
	// when BTF=1, SCL will be stretched (pulled to LOW)
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) );
	while (! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF) );

	// generate STOP condition and master need not to wait for the completion of stop condition
	// generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR) // if not configured as repeated start
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_MasterReceiveData
 *
 *  @brief			-	This function is used for receiving data
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *  @param[in]		-	Slave address
 *  @param[in]		-	Repeated start condition
 *
 *  @return			-	none
 *
 *  @Note			-	This is blocking call
 *
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t Sr)
{
	// Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is completed by checking the SB flag in the SR1
	// until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// send the address of the slave with R/nW bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddress);

	// confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// procedure to read only 1 byte from slave
	if(Len == 1)
	{
		// disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// wait till RXNE becomes 1
		while ( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

		if(Sr == I2C_DISABLE_SR) // if not configured as repeated start
		{
			// generate STOP condition
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// read data buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;
	}

	// procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// read the data until Len becomes 0
		for(uint32_t i = Len ; i > 0 ; i-- )
		{
			// wait till RXNE becomes 1
			while ( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE) );

			// if last 2 byes are remaining
			if(i == 2)
			{
				// clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				if(Sr == I2C_DISABLE_SR) // if not configured as repeated start
				{
					// generate STOP condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			// read the data from data register in to buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			// increment the buffer address
			pRxbuffer++;
		}
	}

	// re-enable Acking
	if(pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_ManageAcking
 *
 *  @brief			-	This function enables or disables ACK bit
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		// enable the ACK
		pI2Cx->CR1 |= 1 << I2C_CR1_ACK;
	}else
	{
		// disable the ACK
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_MasterSendDataIT
 *
 *  @brief			-	This function returns the state of Tx
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *  @param[in]		-	Slave address
 *  @param[in]		-	Repeated start condition
 *
 *  @return			-	State of Tx
 *
 *  @Note			-	none
 *
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddress;
		pI2CHandle->Sr = Sr;

		// Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/***********************************************************************************************************
 *  @fn				-	I2C_MasterReceiveDataIT
 *
 *  @brief			-	This function returns the state of Rx
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *  @param[in]		-	RX buffer
 *  @param[in]		-	Data length
 *  @param[in]		-	Slave address
 *  @param[in]		-	Repeated start condition
 *
 *  @return			-	State of Rx
 *
 *  @Note			-	none
 *
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddress;
		pI2CHandle->Sr = Sr;

		// Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/***********************************************************************************************************
 *  @fn				-	I2C_SlaveSendData
 *
 *  @brief			-	This function loads a data to Data Register
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *  @param[in]		-	Data for sending
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

/***********************************************************************************************************
 *  @fn				-	I2C_SlaveReceiveData
 *
 *  @brief			-	This function loads a data from Data Register
 *
 *  @param[in]		-	Base address of the I2C peripheral
 *
 *  @return			-	Data register value
 *
 *  @Note			-	none
 *
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

/*
 *	IRQ Configuration and ISR Handling
 */
/***********************************************************************************************************
 *  @fn				-	I2C_IRQInterruptConfig
 *
 *  @brief			-	This function configurates an exception
 *
 *  @param[in]		-	IRQ number of an exception
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber < 32)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
	} else {
		if (IRQNumber < 32)
		{
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_IRQPriorityConfig
 *
 *  @brief			-	This function configurates a priority of an exception
 *
 *  @param[in]		-	IRQ number of an exception
 *  @param[in]		-	IRQ priority of an exception
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	// First find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;

	// in each section of IRQ Priority register, first 4 bits are not implemented in STM32F407x.
	// no of priority bits implemented depends on vendor
	uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPRx) |= IRQPriority << shift_amount;
}

/***********************************************************************************************************
 *  @fn				-	I2C_MasterHandleTXEInterrupt
 *
 *  @brief			-	This function is helper function of I2C TXE interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0)
	{
		// load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		// decrement the TxLen
		pI2CHandle->TxLen--;

		// increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_MasterHandleRXNEInterrupt
 *
 *  @brief			-	This function is helper function of I2C RXNE interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the I2C peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	} else if (pI2CHandle->RxSize > 1)
	{
		if (pI2CHandle->RxLen == 2)
		{
			// clear the ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		// read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxLen == 0) {
		// close the I2C data reception and notify the application

		// generate the STOP condition
		if (pI2CHandle->Sr == I2C_DISABLE_SR) {
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// close the I2C RX
		I2C_CloseReceiveData(pI2CHandle);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_CloseSendData
 *
 *  @brief			-	This function closes I2C transmission
 *
 *  @param[in]		-	Base address and user configuration of the I2Cx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/***********************************************************************************************************
 *  @fn				-	I2C_CloseReceiveData
 *
 *  @brief			-	This function closes I2C reception
 *
 *  @param[in]		-	Base address and user configuration of the I2Cx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVTEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_EV_IRQHandling
 *
 *  @brief			-	This function handles an exception of events for a given I2C pin
 *
 *  @param[in]		-	Base address and user configuration of the I2Cx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// interrupt handling for both master and slave mode of a device
	uint8_t temp1, temp2, temp3;

	// 1. Handle for interrupt generated by SB event
	// SB flag is only applicable in Master mode
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	if( temp1 && temp3)
	{
		// the interrupt is generated because of SB event
		// this block will not be executed in slave mode because for slave SB is always zero
		// execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

	}

	// 2. Handle for interrupt generated by ADDR event
	// when master mode: Address is sent
	// when slave mode: Address matched with own address

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	if( temp1 && temp3)
	{
		// ADDR flag is set
		// then clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// 3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	if (temp1 && temp3)
	{
		// BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				// BTF, TXE = 1
				// check the length of data
				if(pI2CHandle->TxLen == 0)
				{
					// generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					// notify the application that the transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}

	// 4. Handle for interrupt generated by STOP event
	// stop detection flag is applicable only slave mode.
	// for master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

	if (temp1 && temp3)
	{
		// STOPF flag is set
		// clear the STOPF: read SR1 (already has been read above) then write CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000; // content of CR1 will not be affected

		// notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// 5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

	if (temp1 && temp2 && temp3)
	{
		// only in master mode. check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// The device is Master

			// TXE flag is set
			// do the data transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			// The device is Slave
			// make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	// 6. Handle for interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

	if (temp1 && temp2 && temp3)
	{
		// only in master mode. check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// The device is Master

			// RXNE flag is set
			// do the data reception
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			// The device is Slave
			// make sure that the slave is really in receiver mode
			if ( ! (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) )
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/***********************************************************************************************************
 *  @fn				-	I2C_ER_IRQHandling
 *
 *  @brief			-	This function handles an exception of errors for a given I2C pin
 *
 *  @param[in]		-	Base address and user configuration of the I2Cx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    // the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		// this is Bus error

		// clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		// this is arbitration lost error

		// clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		// this is ACK failure error

	    // clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		// this is Overrun/underrun

	    // clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		// this is Time out error

	    // clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		// notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}
