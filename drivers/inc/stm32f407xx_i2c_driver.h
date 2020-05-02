/*
 * stm32f407xx_i2c_driver.h
 *
 *      Author: Red Russian Army
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 *	Configuration structure for a I2Cx peripheral
 */
typedef struct{

	uint32_t I2C_SCLSpeed;			/*!< possible values from @I2C_SCLSpeed >*/
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;			/*!< possible values from @I2C_ACKControl >*/
	uint8_t I2C_FMDutyCycle;		/*!< possible values from @I2C_DutyCycle >*/

}I2C_Config_t;


/*
 *	Handle structure for a I2Cx peripheral
 */
typedef struct{

	I2C_RegDef_t *pI2Cx; 			/*!< This holds the base address of I2Cx(x:1,2,3) peripheral >*/
	I2C_Config_t I2CConfig;			/*!< This holds I2Cx peripheral configuration settings >*/
	uint8_t		*pTxBuffer;			/*!< To store the app. Tx buffer address >*/
	uint8_t		*pRxBuffer;			/*!< To store the app. Rx buffer address >*/
	uint32_t	TxLen;				/*!< To store Tx length >*/
	uint32_t	RxLen;				/*!< To store Rx length >*/
	uint8_t		TxRxState;			/*!< To store communication state >*/
	uint8_t		DevAddr;			/*!< To store slave device address >*/
	uint32_t	RxSize;				/*!< To store Rx size >*/
	uint8_t		Sr;					/*!< To store repeated start value >*/

}I2C_Handle_t;

/*
 *	@I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 			100000 		/* standard mode: 100 kHz */
#define I2C_SCL_SPEED_FM4K 			400000		/* fast mode: 400 kHZ */
#define I2C_SCL_SPEED_FM2K			200000		/* fast mode: 200 kHz */

/*
 *	@I2C_ACKControl
 */
#define I2C_ACK_ENABLE				1
#define I2C_ACK_DISABLE				0

/*
 *	@I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_CYCLE_2			0
#define I2C_FM_DUTY_CYCLE_16_9		1

/*
 *	I2C related status flags definitions
 */
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB )
#define I2C_FLAG_ADDR		( 1 << I2C_SR1_ADDR )
#define I2C_FLAG_BTF		( 1 << I2C_SR1_BTF )
#define I2C_FLAG_ADD10		( 1 << I2C_SR1_ADD10 )
#define I2C_FLAG_STOPF		( 1 << I2C_SR1_STOPF )
#define I2C_FLAG_RXNE		( 1 << I2C_SR1_RXNE )
#define I2C_FLAG_TXE		( 1 << I2C_SR1_TXE )
#define I2C_FLAG_BERR		( 1 << I2C_SR1_BERR )
#define I2C_FLAG_ARLO		( 1 << I2C_SR1_ARLO )
#define I2C_FLAG_AF			( 1 << I2C_SR1_AF )
#define I2C_FLAG_OVR		( 1 << I2C_SR1_OVR )
#define I2C_FLAG_PECERR		( 1 << I2C_SR1_PECERR )
#define I2C_FLAG_TIMEOUT	( 1 << I2C_SR1_TIMEOUT )
#define I2C_FLAG_SMBALERT	( 1 << I2C_SR1_SMBALERT )

#define I2C_DISABLE_SR		RESET		/* disable repeated start */
#define I2C_ENABLE_SR		SET			/* enable repeated start */

/*
 *	Possible I2C application states
 */
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/*
 *  I2C application events macros
 */
#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

/*****************************************************************************************
 * 								APIs supported by this driver
 * 				For more information about the APIs, check the function definitions
 *****************************************************************************************/

/*
 *	Peripheral Clock Setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 *	Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 *	Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 *	IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);

/*
 *	Other Peripheral Control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 *	Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
