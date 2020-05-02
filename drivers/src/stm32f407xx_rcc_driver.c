/*
 * stm32f407xx_rcc_driver.c
 *
 *      Author: Red Russian Army
 */

#include "stm32f407xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_PreScaler[8] = {2,4,8,16};
uint16_t APB2_PreScaler[8] = {2,4,8,16};


/***********************************************************************************************************
 *  @fn				-	RCC_GetPLLOutputClock
 *
 *  @brief			-	This function calculates the System PLL Output Clock
 *
 *  @param[in]		-	none
 *
 *  @return			-	PLL Output Clock
 *
 *  @Note			-	none
 *
 */
uint32_t RCC_GetPLLOutputClock()
{
	// it can be implemented to calculate PLL output clock
	return;
}

/***********************************************************************************************************
 *  @fn				-	RCC_GetPCLK1Value
 *
 *  @brief			-	This function reads the PCLK1 value
 *
 *  @param[in]		-	none
 *
 *  @return			-	PCLK1 value
 *
 *  @Note			-	none
 *
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pCLK1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	// used clock source
	clksrc = (RCC->CFGR >> 2) & 0x03;	// to read 2nd and 3rd bits of RCC_CFGR register

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	// according to HPRE bit of RCC_CFGR for AHB
	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp < 8)
	{
		ahbp = 1; 			// AHB bus prescaler value
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	// according to HPRE bit of RCC_CFGR for APB
	temp = (RCC->CFGR >> 10) & 0x07;

	if (temp < 4)
	{
		apb1p = 1; 			// APB1 bus prescaler value
	} else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pCLK1 = (SystemClk / ahbp) / apb1p;

	return pCLK1;
}

/***********************************************************************************************************
 *  @fn				-	RCC_GetPCLK2Value
 *
 *  @brief			-	This function reads the PCLK2 value
 *
 *  @param[in]		-	none
 *
 *  @return			-	PCLK2 value
 *
 *  @Note			-	none
 *
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pCLK2, SystemClk;

	uint8_t clksrc, temp, ahbp, apb2p;

	// used clock source
	clksrc = (RCC->CFGR >> 2) & 0x03;	// to read 2nd and 3rd bits of RCC_CFGR register

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	// according to HPRE bit of RCC_CFGR for AHB
	temp = (RCC->CFGR >> 4) & 0xF;

	if(temp < 8)
	{
		ahbp = 1; 			// AHB bus prescaler value
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	// according to HPRE bit of RCC_CFGR for APB
	temp = (RCC->CFGR >> 13) & 0x07;

	if (temp < 4)
	{
		apb2p = 1; 			// APB2 bus prescaler value
	} else
	{
		apb2p = APB2_PreScaler[temp-4];
	}

	pCLK2 = (SystemClk / ahbp) / apb2p;

	return pCLK2;
}
