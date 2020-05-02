/*
 * stm32f407xx_spi_driver.c
 *
 *      Author: Red Russian Army
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 *	Peripheral Clock Setup
 */
/***********************************************************************************************************
 *  @fn				-	SPI_PeriClockControl
 *
 *  @brief			-	This function enables or disables peripheral clock for SPIx peripheral
 *
 *  @param[in]		-	Base address of the SPI peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*
 *	Init and De-init
 */
/***********************************************************************************************************
 *  @fn				-	SPI_Init
 *
 *  @brief			-	This function configures SPIx Peripheral
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// configure the SPI_CR1 register
	// no need to clear all bits because we initialize a temp register with 0
	uint32_t tempreg = 0;

	//	configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//	configure the bus configuration
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// clear BIDI mode
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// set BIDI
		tempreg |= 1 << SPI_CR1_BIDIMODE;

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// clear BIDI
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

		// set RXONLY
		tempreg |= 1 << SPI_CR1_RXONLY;
	}

	// set clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// set DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// set SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	// set CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// set CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/***********************************************************************************************************
 *  @fn				-	SPI_DeInit
 *
 *  @brief			-	This function resets the register of SPIx Peripheral
 *
 *  @param[in]		-	Base address of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/***********************************************************************************************************
 *  @fn				-	SPI_GetFlagStatus
 *
 *  @brief			-	This function returns flag status for a given flag
 *
 *  @param[in]		-	Base address of the SPI peripheral
 *  @param[in]		-	Flag name
 *
 *  @return			-	Flag status
 *
 *  @Note			-	none
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 *	Data Send and Receive
 */
/***********************************************************************************************************
 *  @fn				-	SPI_SendData
 *
 *  @brief			-	This function is used for sending data
 *
 *  @param[in]		-	Base address of the SPI peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *
 *  @return			-	none
 *
 *  @Note			-	This is blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET );

		// check the DFF bit in CR1
		if (pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			// 16 bit DFF
			// load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;	// Len is decremented twice because 2 bytes of data has been sent
			(uint16_t*)pTxBuffer++;
		}else
		{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/***********************************************************************************************************
 *  @fn				-	SPI_ReceiveData
 *
 *  @brief			-	This function is used for receiving data
 *
 *  @param[in]		-	Base address of the SPI peripheral
 *  @param[in]		-	RX buffer
 *  @param[in]		-	Data length
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET );

		// check the DFF bit in CR1
		if (pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			// 16 bit DFF
			// load the data from the DR to RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;	// Len is decremented twice because 2 bytes of data has been sent
			(uint16_t*)pRxBuffer++;
		}else
		{
			// 8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/***********************************************************************************************************
 *  @fn				-	SPI_PeripheralControl
 *
 *  @brief			-	This function enables or disable a given SPIx peripheral
 *
 *  @param[in]		-	Base address of the SPIx peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/***********************************************************************************************************
 *  @fn				-	SPI_SSOEConfig
 *
 *  @brief			-	This function makes NSS signal internally high and avoids MODF error
 *
 *  @param[in]		-	Base address of the SPIx peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/***********************************************************************************************************
 *  @fn				-	SPI_SSIConfig
 *
 *  @brief			-	This function makes NSS signal internally high and avoids MODF error
 *
 *  @param[in]		-	Base address of the SPIx peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*
 *	IRQ Configuration and ISR Handling
 */
/***********************************************************************************************************
 *  @fn				-	SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 *  @fn				-	SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;

	// in each section of IRQ Priority register, first 4 bits are not implemented in STM32F407x.
	// no of priority bits implemented depends on vendor
	uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPRx) |= IRQPriority << shift_amount;
}

/***********************************************************************************************************
 *  @fn				-	SPI_IRQHandling
 *
 *  @brief			-	This function handles an exception for a given SPI pin
 *
 *  @param[in]		-	pin number
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	// check for TXE
	temp1 = pSPIHandle->pSPIx->SR & ( 1<< SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1<< SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		// handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & ( 1<< SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1<< SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		// handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// check for CRC => not implemented in this driver

	// check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & ( 1<< SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		// handle OVR
		spi_ovr_interrupt_handle(pSPIHandle);
	}
}

/***********************************************************************************************************
 *  @fn				-	SPI_SendDataIT
 *
 *  @brief			-	This function returns state of Tx
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *
 *  @return			-	state of Tx
 *
 *  @Note			-	none
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}

/***********************************************************************************************************
 *  @fn				-	SPI_ReceiveDataIT
 *
 *  @brief			-	This function returns state of Rx
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *
 *  @return			-	state of Rx
 *
 *  @Note			-	none
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}

/***********************************************************************************************************
 *  @fn				-	spi_txe_interrupt_handle
 *
 *  @brief			-	This function is helper function of SPI TXE interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit DFF
		// load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;// Len is decremented twice because 2 bytes of data has been sent
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else
	{
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		// close the SPI communication when TxLen is zero
		// inform the application that TX is over
		// prevent interrupts with setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/***********************************************************************************************************
 *  @fn				-	spi_rxne_interrupt_handle
 *
 *  @brief			-	This function is helper function of SPI RXNE interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit DFF
		// load the data into the DR
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;// Len is decremented twice because 2 bytes of data has been sent
		(uint16_t*) pSPIHandle->pRxBuffer++;
	} else
	{
		// 8 bit DFF
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->RxLen)
	{
		// reception is complete
		// turn off the RXNEIE interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/***********************************************************************************************************
 *  @fn				-	spi_ovr_interrupt_handle
 *
 *  @brief			-	This function is helper function of SPI OVR interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	// clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp; // to prevent unused "temp" variable error

	// inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/***********************************************************************************************************
 *  @fn				-	SPI_ClearOVRFlag
 *
 *  @brief			-	This function clears the OVR flag
 *
 *  @param[in]		-	Base address of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void) temp;
}

/***********************************************************************************************************
 *  @fn				-	SPI_CloseTransmission
 *
 *  @brief			-	This function closes SPI transmission
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/***********************************************************************************************************
 *  @fn				-	SPI_CloseReception
 *
 *  @brief			-	This function closes SPI reception
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/***********************************************************************************************************
 *  @fn				-	SPI_ApplicationEventCallback
 *
 *  @brief			-	This function overrides to Application Event Callback
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral
 *  @param[in]		-	Event name
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// the user application may override this
}
