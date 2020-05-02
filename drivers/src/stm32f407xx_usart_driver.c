/*
 * stm32f407xx_usart_driver.c
 *
 *      Author: Red Russian Army
 */

#include "stm32f407xx_usart_driver.h"

/*
 *	Peripheral Clock Setup
 */
/***********************************************************************************************************
 *  @fn				-	USART_PeriClockControl
 *
 *  @brief			-	This function enables or disables peripheral clock for USARTx peripheral
 *
 *  @param[in]		-	base address of the USART peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		} else if (pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		} else if (pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		} else if (pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}
	} else {
		if (pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		} else if (pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		} else if (pUSARTx == UART7)
		{
			UART7_PCLK_DI();
		} else if (pUSARTx == UART8)
		{
			UART8_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             - This function sets a given baud rate
 *
 * @param[in]         -	base address of the USART peripheral
 * @param[in]         -	Baud rate macro
 *
 * @return            - none
 *
 * @Note              - none

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	// Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	// variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	// get the value of APB bus clock in to the variable PCLKx
	if (pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	} else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	// check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else
	{
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	// Calculate the Mantissa part
	M_part = usartdiv / 100;

	// place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	// extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	// calculate the final fractional
	if (pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1 , over sampling by 8
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t) 0x07);

	} else
	{
		// over sampling by 16
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t) 0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*
 *	Init and De-init
 */
/***********************************************************************************************************
 *  @fn				-	USART_Init
 *
 *  @brief			-	This function configures USARTx Peripheral
 *
 *  @param[in]		-	Base address and user configuration of the USART peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Temporary register
	uint32_t tempreg = 0;

	/******************************** Configuration of CR1******************************************/

	// enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	// enable USART TX and RX engines according to the USART_Mode configuration item
	if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_RX)
	{
		// enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_TX)
	{
		// enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	} else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TXRX)
	{
		// enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	// configure the Word length configuration item
	tempreg |= pUSARTHandle->USARTConfig.USART_WordLength << USART_CR1_M;

	// Configuration of parity control bit fields
	if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		// enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		// enable EVEN parity
		// Not required because by default EVEN parity will be selected

	} else if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		// enable the parity control
		tempreg |= (1 << USART_CR1_PCE);

		// enable ODD parity
		tempreg |= (1 << USART_CR1_PS);

	}

	// Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg = 0;

	// configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USARTConfig.USART_NoOfStopBits << USART_CR2_STOP;

	// Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg = 0;

	// Configuration of USART hardware flow control
	if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		// enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);

	} else if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		// enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	} else if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		// enable both CTS and RTS Flow control
		tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	// configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USARTConfig.USART_Baud);
}

/***********************************************************************************************************
 *  @fn				-	USART_DeInit
 *
 *  @brief			-	This function resets the register of USARTx Peripheral
 *
 *  @param[in]		-	Base address of the USART peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	} else if (pUSARTx == USART2)
	{
		USART2_REG_RESET();
	} else if (pUSARTx == USART3)
	{
		USART3_REG_RESET();
	} else if (pUSARTx == UART4)
	{
		UART4_REG_RESET();
	} else if (pUSARTx == UART5)
	{
		UART5_REG_RESET();
	} else if (pUSARTx == USART6)
	{
		USART6_REG_RESET();
	} else if (pUSARTx == UART7)
	{
		UART7_REG_RESET();
	} else if (pUSARTx == UART8)
	{
		UART8_REG_RESET();
	}
}

/*
 * Data Send and Receive
 */
/***********************************************************************************************************
 *  @fn				-	USART_SendData
 *
 *  @brief			-	This function is used for sending data
 *
 *  @param[in]		-	Base address and user configuration of the USART peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *
 *  @return			-	none
 *
 *  @Note			-	This is blocking call
 *
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	// Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0; i < Len; i++)
	{
		// wait until TXE flag is set in the SR
		while ( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE) );

		// check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

			// check for USART_ParityControl
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used in this transfer. so, 9bits of user data will be sent
				// increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			} else
			{
				// Parity bit is used in this transfer . so , 8bits of user data will be sent
				// The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		} else
		{
			// This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t) 0xFF);

			// increment the buffer address
			pTxBuffer++;
		}
	}

	// wait till TC flag is set in the SR
	while ( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC) );
}

/***********************************************************************************************************
 *  @fn				-	USART_ReceiveData
 *
 *  @brief			-	This function is used for receiving data
 *
 *  @param[in]		-	Base address and user configuration of the USART peripheral
 *  @param[in]		-	RX buffer
 *  @param[in]		-	Data length
 *
 *  @return			-	none
 *
 *  @Note			-	This is blocking call
 *
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	// Loop over until "Len" number of bytes are transferred
	for (uint32_t i = 0; i < Len; i++)
	{
		// wait until RXNE flag is set in the SR
		while ( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE) );

		// Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// We are going to receive 9bit data in a frame

			// check are we using USART_ParityControl control or not
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used. so, all 9bits will be of user data

				// read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);

				// increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			} else
			{
				// Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);

				// increment the pRxBuffer
				pRxBuffer++;
			}
		} else
		{
			// We are going to receive 8bit data in a frame

			// check are we using USART_ParityControl control or not
			if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
			}

			else
			{
				// Parity is used, so , 7 bits will be of user data and 1 bit is parity

				// read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0X7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/***********************************************************************************************************
 *  @fn				-	USART_SendDataIT
 *
 *  @brief			-	This function returns the state of Tx
 *
 *  @param[in]		-	Base address and user configuration of the USART peripheral
 *  @param[in]		-	TX buffer
 *  @param[in]		-	Data length
 *
 *  @return			-	State of Tx
 *
 *  @Note			-	none
 *
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if (txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

/***********************************************************************************************************
 *  @fn				-	USART_ReceiveDataIT
 *
 *  @brief			-	This function returns the state of Rx
 *
 *  @param[in]		-	Base address and user configuration of the USART peripheral
 *  @param[in]		-	RX buffer
 *  @param[in]		-	Data length
 *
 *  @return			-	State of Rx
 *
 *  @Note			-	none
 *
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if (rxstate != pUSARTHandle->RxBusyState)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		// enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}

/*
 *	IRQ Configuration and ISR Handling
 */
/***********************************************************************************************************
 *  @fn				-	USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 *  @fn				-	USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 *  @fn				-	USART_IRQHandling
 *
 *  @brief			-	This function handles an exception for a given USART pin
 *
 *  @param[in]		-	Base address and user configuration of the USARTx peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2, temp3;

	uint16_t *pdata;

	/*************************Check for TC flag ********************************************/

	// check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);

	// check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if (temp1 && temp2)
	{
		// this interrupt is because of TC

		// close transmission and call application callback if TxLen is zero
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// check the TxLen . If it is zero then close the data transmission
			if (!pUSARTHandle->TxLen)
			{
				// clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				// clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);

				// reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				// reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				// reset the length to zero
				pUSARTHandle->TxLen = 0;

				// call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*************************Check for TXE flag ********************************************/

	// check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	// check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if (temp1 && temp2) {
		// this interrupt is because of TXE

		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// keep sending data until Txlen reaches to zero
			if (pUSARTHandle->TxLen > 0)
			{
				// check the USART_WordLength item for 9BIT or 8BIT in a frame
				if (pUSARTHandle->USARTConfig.USART_WordLength	== USART_WORDLEN_9BITS)
				{
					// if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					// loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t) 0x01FF);

					// check for USART_ParityControl
					if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// no parity is used in this transfer, so 9bits of user data will be sent
						// increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						// decrement the length
						pUSARTHandle->TxLen-=2;
					} else
					{
						// parity bit is used in this transfer. so , 8bits of user data will be sent
						// the 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						// to decrement the length
						pUSARTHandle->TxLen--;
					}
				} else
				{
					// this is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t) 0xFF);

					// increment the buffer address
					pUSARTHandle->pTxBuffer++;

					// decrement the length
					pUSARTHandle->TxLen-=1;
				}

			}
			if (pUSARTHandle->TxLen == 0)
			{
				// TxLen is zero
				// clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if (temp1 && temp2) {
		// this interrupt is because of RXNE

		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if (pUSARTHandle->RxLen > 0)
			{
				// check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if (pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// we are going to receive 9bit data in a frame

					// now, check are we using USART_ParityControl control or not
					if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// no parity is used. so, all 9bits will be of user data

						// read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t) 0x01FF);

						// now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						// decrement the length
						pUSARTHandle->RxLen-=2;
					} else
					{
						// parity is used. so, 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);

						// now increment the pRxBuffer
						pUSARTHandle->pRxBuffer++;

						// decrement the length
						pUSARTHandle->RxLen--;
					}
				} else
				{
					// we are going to receive 8bit data in a frame

					// now, check are we using USART_ParityControl control or not
					if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// no parity is used , so all 8bits will be of user data

						// read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0xFF);
					}

					else
					{
						// parity is used, so , 7 bits will be of user data and 1 bit is parity

						// read only 7 bits , hence mask the DR with 0x7F
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t) 0x7F);
					}

					// now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					// decrement the length
					pUSARTHandle->RxLen--;
				}
			}			//if of >0

			if (!pUSARTHandle->RxLen)
			{
				//disable the RXNE
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	/*************************Check for CTS flag ********************************************/
	// Note : CTS feature is not applicable for UART4 and UART5
	// check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	// check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	// check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if (temp1 && temp2)
	{
		// clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_CTS);

		//this interrupt is because of CTS
		USART_ApplicationEventCallback( pUSARTHandle, USART_EVENT_CTS);
	}

	/*************************Check for IDLE detection flag ********************************************/

	// check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);

	// check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	if (temp1 && temp2)
	{
		// clear the IDLE flag. Refer to the RM to understand the clear sequence
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		// this interrupt is because of IDLE
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	/*************************Check for Overrun detection flag ********************************************/

	// check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	// check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;

	if (temp1 && temp2) {
		// need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		// this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_ORE);
	}

	/*************************Check for Error Flag ********************************************/

	// Noise Flag, Overrun error and Framing Error in multibuffer communication
	// The blow code will get executed in only if multibuffer mode is used.
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if (temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if (temp1 & (1 << USART_SR_FE))
		{
			/*
			 This bit is set by hardware when a de-synchronization, excessive noise or a break character
			 is detected. It is cleared by a software sequence (an read to the USART_SR register
			 followed by a read to the USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_FE);
		}

		if (temp1 & (1 << USART_SR_NF))
		{
			/*
			 This bit is set by hardware when noise is detected on a received frame. It is cleared by a
			 software sequence (an read to the USART_SR register followed by a read to the
			 USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_NE);
		}

		if (temp1 & (1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERROR_ORE);
		}
	}
}

/***********************************************************************************************************
 *  @fn				-	USART_PeripheralControl
 *
 *  @brief			-	This function enables or disable a given USARTx peripheral
 *
 *  @param[in]		-	Base address of the USARTx peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	} else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/***********************************************************************************************************
 *  @fn				-	USART_GetFlagStatus
 *
 *  @brief			-	This function returns flag status for a given flag
 *
 *  @param[in]		-	Base address of the USART peripheral
 *  @param[in]		-	Flag name
 *
 *  @return			-	Flag status
 *
 *  @Note			-	none
 *
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***********************************************************************************************************
 *  @fn				-	USART_ClearFlag
 *
 *  @brief			-	This function clears a given flag
 *
 *  @param[in]		-	Base address of the USART peripheral
 *  @param[in]		-	Flag name
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->SR &= ~( StatusFlagName );
}

/*
 * Application callback
 */
/***********************************************************************************************************
 *  @fn				-	USART_ApplicationEventCallback
 *
 *  @brief			-	This function overrides to Application Event Callback
 *
 *  @param[in]		-	Base address and user configuration of the USARTx peripheral
 *  @param[in]		-	Event name
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{

}
