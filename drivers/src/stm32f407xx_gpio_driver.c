/*
 * stm32f407xx_gpio_driver.c
 *
 *      Author: Red Russian Army
 */

#include "stm32f407xx_gpio_driver.h"

/*
 *	Peripheral Clock Setup
 */

/***********************************************************************************************************
 *  @fn				-	GPIO_PeriClockControl
 *
 *  @brief			-	This function enables or disables peripheral clock for a given GPIO port
 *
 *  @param[in]		-	Base address of the GPIO peripheral
 *  @param[in]		-	ENABLE or DISABLE macros
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){

		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}else{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}


/*
 *	Init and De-init
 */
/***********************************************************************************************************
 *  @fn				-	GPIO_Init
 *
 *  @brief			-	This function configures a given GPIO pin
 *
 *  @param[in]		-	Base address and user configuration of the GPIO peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// temporary register
	uint32_t temp = 0;

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//	1. configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //	Multiply with 2 because each pin takes 2 bit fields in MODER register.
		pGPIOHandle->pGPIOx->MODER &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		pGPIOHandle->pGPIOx->MODER |= temp;

	}else
	{
		// interrupt mode
		if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT )
		{
			// configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT )
		{
			// configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
		{
			// configure both FTSR and RTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		// configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( 4 * temp2 );

		// enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}

	temp = 0;

	//	2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//	3. configure the PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//	4. configure the OPTYPE
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//	5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1;

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >= 8 )
		{
			temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8)));
			pGPIOHandle->pGPIOx->AFRH &= ~( 0xF << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8) );
			pGPIOHandle->pGPIOx->AFRH |= temp1;
		}else
		{
			temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL &= ~( 0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			pGPIOHandle->pGPIOx->AFRL |= temp1;
		}
	}

}

/***********************************************************************************************************
 *  @fn				-	GPIO_DeInit
 *
 *  @brief			-	This function resets the register for a given GPIO port
 *
 *  @param[in]		-	Base address of the GPIO peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 *	Data read and write
 */

/***********************************************************************************************************
 *  @fn				-	GPIO_ReadFromInputPin
 *
 *  @brief			-	This function reads a value of GPIO pin for a given GPIO port and pin number
 *
 *  @param[in]		-	Base address of the GPIO peripheral
 *  @param[in]		-	Pin number
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/***********************************************************************************************************
 *  @fn				-	GPIO_ReadFromInputPort
 *
 *  @brief			-	This function reads the given GPIO port
 *
 *  @param[in]		-	Base address of the GPIO peripheral
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/***********************************************************************************************************
 *  @fn				-	GPIO_WriteToOutputPin
 *
 *  @brief			-	This function writes a value for a given GPIO port
 *
 *  @param[in]		-	Base address of the GPIO peripheral
 *  @param[in]		-	Pin number
 *  @param[in]		-	Output value
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= ( 1 << PinNumber );
	}else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber );
	}
}

/***********************************************************************************************************
 *  @fn				-	GPIO_WriteToOutputPort
 *
 *  @brief			-	This function writes to register for a given GPIO port
 *
 *  @param[in]		-	Base address of the GPIO peripheral
 *  @param[in]		-	Output value
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***********************************************************************************************************
 *  @fn				-	GPIO_ToggleOutputPin
 *
 *  @brief			-	This function toggles a pin for a given GPIO port
 *
 *  @param[in]		-	Base address of the GPIO peripheral
 *  @param[in]		-	Pin number
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber );
}


/*
 *	IRQ Interrupt Configuration and ISR Handling
 */

/***********************************************************************************************************
 *  @fn				-	GPIO_IRQInterruptConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber < 32)
		{
			// program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			// program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if (IRQNumber < 32)
		{
			// program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		}else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );

		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/*
 *	IRQ Priority Configuration and ISR Handling
 */

/***********************************************************************************************************
 *  @fn				-	GPIO_IRQPriorityConfig
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
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
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
 *  @fn				-	GPIO_IRQHandling
 *
 *  @brief			-	This function handles an exception for a given GPIO pin
 *
 *  @param[in]		-	Pin number
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI PR register corresponding to the pin number
	if( EXTI->PR & (1 << PinNumber) )
	{
		// clearing pending register means setting related bit as 1
		EXTI->PR |= (1 << PinNumber);
	}
}
