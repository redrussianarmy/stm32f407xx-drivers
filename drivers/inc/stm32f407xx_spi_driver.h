/*
 * stm32f407xx_spi_driver.h
 *
 *      Author: Red Russian Army
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 *	Configuration structure for a SPIx peripheral
 */
typedef struct{

	uint8_t SPI_DeviceMode;			/*!< possible values from @SPI_DeviceMode >*/
	uint8_t SPI_BusConfig;			/*!< possible values from @SPI_BusConfig >*/
	uint8_t SPI_SclkSpeed;			/*!< possible values from @SPI_SclkSpeed >*/
	uint8_t SPI_DFF;				/*!< possible values from @SPI_DFF >*/
	uint8_t SPI_CPHA;				/*!< possible values from @SPI_CPHA >*/
	uint8_t SPI_CPOL;				/*!< possible values from @SPI_CPOL >*/
	uint8_t SPI_SSM;				/*!< possible values from @SPI_SSM >*/

}SPI_Config_t;

/*
 *	Handle structure for a SPIx peripheral
 */
typedef struct{

	SPI_RegDef_t *pSPIx; 			/*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
	SPI_Config_t SPIConfig;			/*!< This holds SPIx peripheral configuration settings >*/
	uint8_t		*pTxBuffer;			/*!< To store the app. Tx buffer address >*/
	uint8_t		*pRxBuffer;			/*!< To store the app. Rx buffer address >*/
	uint32_t	TxLen;				/*!< To store Tx length >*/
	uint32_t	RxLen;				/*!< To store Rx length >*/
	uint8_t		TxState;			/*!< To store Tx state >*/
	uint8_t		RxState;			/*!< To store Rx state >*/

}SPI_Handle_t;

/*
 *	@SPI_DeviceMode
 *	SPI device mode: master, slave
 */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/*
 *	@SPI_BusConfig
 *	SPI bus configuration: full-duplex, half-duplex, simplex (TX only, RX only)
 *	(according to BIDIMODE, BIDIOE and RXONLY fields in CR1 register)
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
//	#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	- not needed an extra option. It can be configured automatically when MOSI line is not connected
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 *	@SPI_SclkSpeed
 *	SPI clock speed (according to BR-Baud Rate field in CR1 register)
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 *	@SPI_DFF
 *	SPI data frame format: 8-bit, 16-bit shift register
 */
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/*
 *	@SPI_CPOL
 *	Clock polarity
 */
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

/*
 *	@SPI_CPHA
 *	Clock phase
 */
#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

/*
 *	@SPI_SSM
 *	Slave select management
 */
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

/*
 *	SPI related status flags definitions
 */
#define SPI_FLAG_RXNE		( 1 << SPI_SR_RXNE )
#define SPI_FLAG_TXE		( 1 << SPI_SR_TXE )
#define SPI_FLAG_CHSIDE		( 1 << SPI_SR_CHSIDE )
#define SPI_FLAG_UDR		( 1 << SPI_SR_UDR )
#define SPI_FLAG_CRCERR		( 1 << SPI_SR_CRCERR )
#define SPI_FLAG_MODF		( 1 << SPI_SR_MODF )
#define SPI_FLAG_OVR		( 1 << SPI_SR_OVR )
#define SPI_FLAG_BSY		( 1 << SPI_SR_BSY )
#define SPI_FLAG_FRE		( 1 << SPI_SR_FRE )

/*
 *	Possible SPI application states
 */
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 *	Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

/*****************************************************************************************
 * 								APIs supported by this driver
 * 				For more information about the APIs, check the function definitions
 *****************************************************************************************/
/*
 *	Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 *	Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx); // register backs to its reset value

/*
 *	Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 *	IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 *	Other Peripheral Control APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 *	Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
