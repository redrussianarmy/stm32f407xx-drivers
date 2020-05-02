/*
 * stm32f407xx.h
 *
 *      Author: Red Russian Army
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo	volatile
#define __weak	__attribute__((weak))

/*****************************************************************************************
 *
 * 						     Processor Specific Details
 *
 *****************************************************************************************/
/*
 *	ARM Cortes Mx Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0 			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 			((__vo uint32_t*)0xE000E10C)
// rest of ISER registers were not implemented

/*
 *	ARM Cortes Mx Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1 			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2 			((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER3 			((__vo uint32_t*)0xE000E18C)
// rest of ICER registers were not implemented

/*
 *	ARM Cortes Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED		4							/* Implemented priority bits in STM32F407xx */

/*
 *	Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR		0x08000000U							/* Base address of FLASH memory */
#define SRAM1_BASEADDR		0x20000000U							/* Base address of SRAM1 */
#define SRAM2_BASEADDR		0x2001C000U							/* Base address of SRAM2 */
#define ROM_BASEADDR		0x1FFF0000U							/* Base address of ROM */
#define SRAM 				SRAM1_BASEADDR						/* Base address of SRAM */

/*
 *  Base addresses of AHBx and APBx Bus Peripheral
 */
#define PERIPH_BASEADDR			0x40000000U						/* Base address of peripherals */
#define APB1_PERIPH_BASEADDR	PERIPH_BASEADDR					/* Base address of APB1 bus peripherals */
#define APB2_PERIPH_BASEADDR	0x40010000U						/* Base address of APB2 bus peripherals */
#define AHB1_PERIPH_BASEADDR	0x40020000U						/* Base address of AHB1 bus peripherals */
#define AHB2_PERIPH_BASEADDR	0x50000000U						/* Base address of AHB2 bus peripherals */

/*
 *	Base addresses of peripherals which are hanging on AHB1 bus
 */
#define RCC_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x3800)		/* Base address of RCC */

#define GPIOA_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x0000)		/* Base address of GPIOA pins */
#define GPIOB_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x0400)		/* Base address of GPIOB pins */
#define GPIOC_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x0800)		/* Base address of GPIOC pins */
#define GPIOD_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x0C00)		/* Base address of GPIOD pins */
#define GPIOE_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x1000)		/* Base address of GPIOE pins */
#define GPIOF_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x1400)		/* Base address of GPIOF pins */
#define GPIOG_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x1800)		/* Base address of GPIOG pins */
#define GPIOH_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x1C00)		/* Base address of GPIOH pins */
#define GPIOI_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x2000)		/* Base address of GPIOI pins */
#define GPIOJ_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x2400)		/* Base address of GPIOJ pins */
#define GPIOK_BASEADDR		(AHB1_PERIPH_BASEADDR + 0x2800)		/* Base address of GPIOK pins */

/*
 *	Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR		(APB1_PERIPH_BASEADDR + 0x5400)		/* Base address of I2C1 */
#define I2C2_BASEADDR		(APB1_PERIPH_BASEADDR + 0x5800)		/* Base address of I2C2 */
#define I2C3_BASEADDR		(APB1_PERIPH_BASEADDR + 0x5C00)		/* Base address of I2C3 */

#define SPI2_BASEADDR		(APB1_PERIPH_BASEADDR + 0x3800)		/* Base address of SPI2 */
#define SPI3_BASEADDR		(APB1_PERIPH_BASEADDR + 0x3C00)		/* Base address of SPI3 */

#define USART2_BASEADDR		(APB1_PERIPH_BASEADDR + 0x4400)		/* Base address of USART2 */
#define USART3_BASEADDR		(APB1_PERIPH_BASEADDR + 0x4800)		/* Base address of USART3 */
#define UART4_BASEADDR		(APB1_PERIPH_BASEADDR + 0x4C00)		/* Base address of UART4 */
#define UART5_BASEADDR		(APB1_PERIPH_BASEADDR + 0x5000)		/* Base address of UART5 */
#define UART7_BASEADDR		(APB1_PERIPH_BASEADDR + 0x7800)		/* Base address of UART7 */
#define UART8_BASEADDR		(APB1_PERIPH_BASEADDR + 0x7C00)		/* Base address of UART8 */

/*
 *	Base addresses of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASEADDR		(APB2_PERIPH_BASEADDR + 0x3000)		/* Base address of SPI1 */
#define SPI4_BASEADDR		(APB2_PERIPH_BASEADDR + 0x3400)		/* Base address of SPI4 */
#define SPI5_BASEADDR		(APB2_PERIPH_BASEADDR + 0x5000)		/* Base address of SPI5 */
#define SPI6_BASEADDR		(APB2_PERIPH_BASEADDR + 0x5400)		/* Base address of SPI6 */

#define USART1_BASEADDR		(APB2_PERIPH_BASEADDR + 0x1000)		/* Base address of USART1 */
#define USART6_BASEADDR		(APB2_PERIPH_BASEADDR + 0x1400)		/* Base address of USART6 */

#define EXTI_BASEADDR		(APB2_PERIPH_BASEADDR + 0x3C00)		/* Base address of EXTI */
#define SYSCFG_BASEADDR		(APB2_PERIPH_BASEADDR + 0x3800)		/* Base address of SYSCFG */

/*
 *	Peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;		/* GPIO port mode register					Address offset: 0x00 */
	__vo uint32_t OTYPER;		/* GPIO port output type register			Address offset: 0x04 */
	__vo uint32_t OSPEEDR;		/* GPIO port output speed register			Address offset: 0x08 */
	__vo uint32_t PUPDR;		/* GPIO port pull-up/pull-down register		Address offset: 0x0C */
	__vo uint32_t IDR;			/* GPIO port input data register			Address offset: 0x10 */
	__vo uint32_t ODR;			/* GPIO port output data register			Address offset: 0x14 */
	__vo uint32_t BSRR;			/* GPIO port bit set/reset register 		Address offset: 0x18 */
	__vo uint32_t LCKR;			/* GPIO port configuration lock register 	Address offset: 0x1C */
	__vo uint32_t AFRL;			/* GPIO alternate function low register 	Address offset: 0x20 */
	__vo uint32_t AFRH;			/* GPIO alternate function high register	Address offset: 0x24 */
} GPIO_RegDef_t;

/*
 *	Peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;			/* SPI control register 1				Address offset: 0x00 */
	__vo uint32_t CR2;			/* SPI control register 2				Address offset: 0x04 */
	__vo uint32_t SR;			/* SPI status register					Address offset: 0x08 */
	__vo uint32_t DR;			/* SPI data register					Address offset: 0x0C */
	__vo uint32_t CRCPR;		/* SPI CRC polynomial register			Address offset: 0x10 */
	__vo uint32_t RXXCRCR;		/* SPI RX CRC register					Address offset: 0x14 */
	__vo uint32_t TXCRCR;		/* SPI TX CRC register 					Address offset: 0x18 */
	__vo uint32_t I2SCFGR;		/* SPI_I2S configuration register		Address offset: 0x1C */
	__vo uint32_t I2SPR;		/* SPI_I2S prescaler register 			Address offset: 0x20 */
} SPI_RegDef_t;

/*
 *	Peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;			/* I2C Control register 1				Address offset: 0x00 */
	__vo uint32_t CR2;			/* I2C Control register 2				Address offset: 0x04 */
	__vo uint32_t OAR1;			/* I2C Own address register 1			Address offset: 0x08 */
	__vo uint32_t OAR2;			/* I2C Own address register 2			Address offset: 0x0C */
	__vo uint32_t DR;			/* I2C Data Register					Address offset: 0x10 */
	__vo uint32_t SR1;			/* I2C Status register 1				Address offset: 0x14 */
	__vo uint32_t SR2;			/* I2C Status register 2				Address offset: 0x18 */
	__vo uint32_t CCR;			/* I2C Clock control register			Address offset: 0x1C */
	__vo uint32_t TRISE;		/* I2C TRISE register					Address offset: 0x20 */
	__vo uint32_t FLTR;			/* I2C FLTR register					Address offset: 0x24 */
}I2C_RegDef_t;

/*
 *	Peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;			/* USART Status register						Address offset: 0x00 */
	__vo uint32_t DR;			/* USART Data register							Address offset: 0x04 */
	__vo uint32_t BRR;			/* USART Baud rate register						Address offset: 0x08 */
	__vo uint32_t CR1;			/* USART Control register 1						Address offset: 0x0C */
	__vo uint32_t CR2;			/* USART Control register 2						Address offset: 0x10 */
	__vo uint32_t CR3;			/* USART Control register 3						Address offset: 0x14 */
	__vo uint32_t GTPR;			/* USART Guard time and prescaler register		Address offset: 0x18 */
}USART_RegDef_t;

/*
 *	Peripheral register definition structure for RCC
 */
typedef struct
{
	__vo uint32_t CR;			/* RCC clock control register									Address offset: 0x00 */
	__vo uint32_t PLLCFGR;		/* RCC PLL configuration register								Address offset: 0x04 */
	__vo uint32_t CFGR;			/* RCC clock configuration register								Address offset: 0x08 */
	__vo uint32_t CIR;			/* RCC clock interrupt register									Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;		/* RCC AHB1 peripheral reset register							Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;		/* RCC AHB2 peripheral reset register							Address offset: 0x14 */
	__vo uint32_t AHB3RSTR;		/* RCC AHB3 peripheral reset register 							Address offset: 0x18 */
	uint32_t 	  RESERVED0;	/* Reserved, 0x1C */
	__vo uint32_t APB1RSTR;		/* RCC APB1 peripheral reset register 							Address offset: 0x20 */
	__vo uint32_t APB2RSTR;		/* RCC APB2 peripheral reset register 							Address offset: 0x24 */
	uint32_t 	  RESERVED1;	/* Reserved, 0x28 */
	uint32_t 	  RESERVED2;	/* Reserved, 0x2C */
	__vo uint32_t AHB1ENR;		/* RCC AHB1 peripheral clock enable register					Address offset: 0x30 */
	__vo uint32_t AHB2ENR;		/* RCC AHB2 peripheral clock enable register					Address offset: 0x34 */
	__vo uint32_t AHB3ENR;		/* RCC AHB3 peripheral clock enable register					Address offset: 0x38 */
	uint32_t 	  RESERVED3;	/* Reserved, 0x3C */
	__vo uint32_t APB1ENR;		/* RCC APB1 peripheral clock enable register					Address offset: 0x40 */
	__vo uint32_t APB2ENR;		/* RCC APB2 peripheral clock enable register					Address offset: 0x44 */
	uint32_t 	  RESERVED4;	/* Reserved, 0x48 */
	uint32_t 	  RESERVED5;	/* Reserved, 0x4C */
	__vo uint32_t AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register	Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register	Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;	/* RCC AHB3 peripheral clock enable in low power mode register	Address offset: 0x58 */
	uint32_t 	  RESERVED6;	/* Reserved, 0x5C */
	__vo uint32_t APB1LPENR;	/* RCC APB1 peripheral clock enable in low power mode register	Address offset: 0x60 */
	__vo uint32_t APB2LPENR;	/* RCC APB2 peripheral clock enabled in low power mode register	Address offset: 0x64 */
	uint32_t 	  RESERVED7;	/* Reserved, 0x68 */
	uint32_t 	  RESERVED8;	/* Reserved, 0x6C */
	__vo uint32_t BDCR;			/* RCC Backup domain control register							Address offset: 0x70 */
	__vo uint32_t CSR;			/* RCC clock control & status register							Address offset: 0x74 */
	uint32_t 	  RESERVED9;	/* Reserved, 0x78 */
	uint32_t 	  RESERVED10;	/* Reserved, 0x7C */
	__vo uint32_t SSCGR;		/* RCC spread spectrum clock generation register				Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register							Address offset: 0x84 */
} RCC_RegDef_t;

/*
 *	Peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;			/* Interrupt mask register					Address offset: 0x00 */
	__vo uint32_t EMR;			/* Event mask register						Address offset: 0x04 */
	__vo uint32_t RTSR;			/* Rising trigger selection register		Address offset: 0x08 */
	__vo uint32_t FTSR;			/* Falling trigger selection register		Address offset: 0x0C */
	__vo uint32_t SWIER;		/* Software interrupt event register		Address offset: 0x10 */
	__vo uint32_t PR;			/* Pending register							Address offset: 0x14 */
}EXTI_RegDef_t;

/*
 *	Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;		/* SYSCFG memory remap register											Address offset: 0x00 */
	__vo uint32_t PMC;			/* SYSCFG peripheral mode configuration register						Address offset: 0x04 */
	__vo uint32_t EXTICR[4];	/* SYSCFG external interrupt configuration register 1, 2, 3 and 4		Address offset: 0x08 - 0x14 */
	uint32_t	RESERVED0;		/* Reserved																Address offset: 0x18 */
	uint32_t	RESERVED1;		/* Reserved																Address offset: 0x1C */
	__vo uint32_t CMPCR;		/* Compensation cell control register									Address offset: 0x20 */
}SYSCFG_RegDef_t;

/*
 * 	Peripheral definitions (Peripheral base addresses typecasted to Related Register Definition Structure)
 */
#define GPIOA 	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 	((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 	((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 	((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 	((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ 	((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK 	((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define SPI1 	((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 	((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 	((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 	((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5 	((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6 	((SPI_RegDef_t*)SPI6_BASEADDR)

#define I2C1 	((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2 	((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3 	((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1 	((USART_RegDef_t*)USART1_BASEADDR)
#define USART2 	((USART_RegDef_t*)USART2_BASEADDR)
#define USART3 	((USART_RegDef_t*)USART3_BASEADDR)
#define UART4 	((USART_RegDef_t*)UART4_BASEADDR)
#define UART5 	((USART_RegDef_t*)UART5_BASEADDR)
#define USART6 	((USART_RegDef_t*)USART6_BASEADDR)
#define UART7 	((USART_RegDef_t*)UART7_BASEADDR)
#define UART8 	((USART_RegDef_t*)UART8_BASEADDR)

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 *	Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()		RCC->AHB1ENR |= (1 << 0) 		/* Enables GPIOA clock */
#define GPIOB_PCLK_EN()		RCC->AHB1ENR |= (1 << 1)		/* Enables GPIOB clock */
#define GPIOC_PCLK_EN()		RCC->AHB1ENR |= (1 << 2)		/* Enables GPIOC clock */
#define GPIOD_PCLK_EN()		RCC->AHB1ENR |= (1 << 3)		/* Enables GPIOD clock */
#define GPIOE_PCLK_EN()		RCC->AHB1ENR |= (1 << 4)		/* Enables GPIOE clock */
#define GPIOF_PCLK_EN()		RCC->AHB1ENR |= (1 << 5)		/* Enables GPIOF clock */
#define GPIOG_PCLK_EN()		RCC->AHB1ENR |= (1 << 6)		/* Enables GPIOG clock */
#define GPIOH_PCLK_EN()		RCC->AHB1ENR |= (1 << 7)		/* Enables GPIOH clock */
#define GPIOI_PCLK_EN()		RCC->AHB1ENR |= (1 << 8)		/* Enables GPIOI clock */

/*
 *	Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		RCC->APB1ENR |= (1 << 21) 		/* Enables I2C1 clock */
#define I2C2_PCLK_EN()		RCC->APB1ENR |= (1 << 22) 		/* Enables I2C2 clock */
#define I2C3_PCLK_EN()		RCC->APB1ENR |= (1 << 23) 		/* Enables I2C2 clock */

/*
 *	Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		RCC->APB2ENR |= (1 << 12) 		/* Enables SPI2 clock */
#define SPI2_PCLK_EN()		RCC->APB1ENR |= (1 << 14) 		/* Enables SPI2 clock */
#define SPI3_PCLK_EN()		RCC->APB1ENR |= (1 << 15) 		/* Enables SPI2 clock */

/*
 *	Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()	RCC->APB2ENR |= (1 << 4) 		/* Enables USART1 clock */
#define USART2_PCLK_EN()	RCC->APB1ENR |= (1 << 17) 		/* Enables USART2 clock */
#define USART3_PCLK_EN()	RCC->APB1ENR |= (1 << 18) 		/* Enables USART3 clock */
#define UART4_PCLK_EN()		RCC->APB1ENR |= (1 << 19) 		/* Enables UART4 clock */
#define UART5_PCLK_EN()		RCC->APB1ENR |= (1 << 20) 		/* Enables UART5 clock */
#define USART6_PCLK_EN()	RCC->APB2ENR |= (1 << 5) 		/* Enables USART6 clock */
#define UART7_PCLK_EN()		RCC->APB1ENR |= (1 << 30) 		/* Enables UART7 clock */
#define UART8_PCLK_EN()		RCC->APB1ENR |= (1 << 31) 		/* Enables UART8 clock */

/*
 *	Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()	RCC->APB2ENR |= (1 << 14) 		/* Enables SYSCFG clock */

/*
 *	Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 0) 		/* Disables GPIOA clock */
#define GPIOB_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 1)		/* Disables GPIOB clock */
#define GPIOC_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 2)		/* Disables GPIOC clock */
#define GPIOD_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 3)		/* Disables GPIOD clock */
#define GPIOE_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 4)		/* Disables GPIOE clock */
#define GPIOF_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 5)		/* Disables GPIOF clock */
#define GPIOG_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 6)		/* Disables GPIOG clock */
#define GPIOH_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 7)		/* Disables GPIOH clock */
#define GPIOI_PCLK_DI()		RCC->AHB1ENR &= ~(1 << 8)		/* Disables GPIOI clock */

/*
 *	Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		RCC->APB1ENR &= ~(1 << 21) 		/* Disables I2C1 clock */
#define I2C2_PCLK_DI()		RCC->APB1ENR &= ~(1 << 22) 		/* Disables I2C2 clock */
#define I2C3_PCLK_DI()		RCC->APB1ENR &= ~(1 << 23) 		/* Disables I2C2 clock */

/*
 *	Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		RCC->APB2ENR &= ~(1 << 12) 		/* Disables SPI2 clock */
#define SPI2_PCLK_DI()		RCC->APB1ENR &= ~(1 << 14) 		/* Disables SPI2 clock */
#define SPI3_PCLK_DI()		RCC->APB1ENR &= ~(1 << 15) 		/* Disables SPI2 clock */

/*
 *	Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()	RCC->APB2ENR &= ~(1 << 4) 		/* Disables USART1 clock */
#define USART2_PCLK_DI()	RCC->APB1ENR &= ~(1 << 17) 		/* Disables USART2 clock */
#define USART3_PCLK_DI()	RCC->APB1ENR &= ~(1 << 18) 		/* Disables USART3 clock */
#define UART4_PCLK_DI()		RCC->APB1ENR &= ~(1 << 19) 		/* Disables UART4 clock */
#define UART5_PCLK_DI()		RCC->APB1ENR &= ~(1 << 20) 		/* Disables UART5 clock */
#define USART6_PCLK_DI()	RCC->APB2ENR &= ~(1 << 5) 		/* Disables USART6 clock */
#define UART7_PCLK_DI()		RCC->APB1ENR &= ~(1 << 30) 		/* Disables UART7 clock */
#define UART8_PCLK_DI()		RCC->APB1ENR &= ~(1 << 31) 		/* Disables UART8 clock */

/*
 *	Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()	RCC->APB2ENR &= ~(1 << 14) 		/* Disables SYSCFG clock */

/*
 *	Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0)); }	while(0)	/* Sets GPIOA reset pin and then Resets */
#define GPIOB_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1)); }	while(0)	/* Sets GPIOB reset pin and then Resets */
#define GPIOC_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2)); }	while(0)	/* Sets GPIOC reset pin and then Resets */
#define GPIOD_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3)); }	while(0)	/* Sets GPIOD reset pin and then Resets */
#define GPIOE_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4)); }	while(0)	/* Sets GPIOE reset pin and then Resets */
#define GPIOF_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5)); }	while(0)	/* Sets GPIOF reset pin and then Resets */
#define GPIOG_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6)); }	while(0)	/* Sets GPIOG reset pin and then Resets */
#define GPIOH_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7)); }	while(0)	/* Sets GPIOH reset pin and then Resets */
#define GPIOI_REG_RESET()	do { (RCC->AHB1RSTR |= (1 << 8));	(RCC->AHB1RSTR &= ~(1 << 8)); }	while(0)	/* Sets GPIOI reset pin and then Resets */

#define SPI1_REG_RESET()	do { (RCC->APB2RSTR |= (1 << 12));	(RCC->APB2RSTR &= ~(1 << 12)); }while(0)	/* Sets SPI1 reset pin and then Resets */
#define SPI2_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 14));	(RCC->APB1RSTR &= ~(1 << 14)); }while(0)	/* Sets SPI2 reset pin and then Resets */
#define SPI3_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 15));	(RCC->APB1RSTR &= ~(1 << 15)); }while(0)	/* Sets SPI3 reset pin and then Resets */

#define I2C1_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 21));	(RCC->APB1RSTR &= ~(1 << 21)); }while(0)	/* Sets I2C1 reset pin and then Resets */
#define I2C2_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 22));	(RCC->APB1RSTR &= ~(1 << 22)); }while(0)	/* Sets I2C2 reset pin and then Resets */
#define I2C3_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 23));	(RCC->APB1RSTR &= ~(1 << 23)); }while(0)	/* Sets I2C3 reset pin and then Resets */

#define USART1_REG_RESET()	do { (RCC->APB2RSTR |= (1 << 4));	(RCC->APB2RSTR &= ~(1 << 4)); }	while(0)	/* Sets USART1 reset pin and then Resets */
#define USART2_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 17));	(RCC->APB1RSTR &= ~(1 << 17)); }while(0)	/* Sets USART2 reset pin and then Resets */
#define USART3_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 18));	(RCC->APB1RSTR &= ~(1 << 18)); }while(0)	/* Sets USART3 reset pin and then Resets */
#define UART4_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 19));	(RCC->APB1RSTR &= ~(1 << 19)); }while(0)	/* Sets UART4 reset pin and then Resets */
#define UART5_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 20));	(RCC->APB1RSTR &= ~(1 << 20)); }while(0)	/* Sets UART5 reset pin and then Resets */
#define USART6_REG_RESET()	do { (RCC->APB2RSTR |= (1 << 5));	(RCC->APB1RSTR &= ~(1 << 5)); }	while(0)	/* Sets USART6 reset pin and then Resets */
#define UART7_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 30));	(RCC->APB1RSTR &= ~(1 << 30)); }while(0)	/* Sets UART7 reset pin and then Resets */
#define UART8_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 31));	(RCC->APB1RSTR &= ~(1 << 31)); }while(0)	/* Sets UART8 reset pin and then Resets */

/*
 * 	Returns port code for a given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)  (	(x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 : 0 )

/*
 *	IRQ numbers of STM32F407x
 */
#define IRQ_NO_EXTI0		6		/* IRQ (Interrupt Request) number of EXTI0 */
#define IRQ_NO_EXTI1		7		/* IRQ (Interrupt Request) number of EXTI1 */
#define IRQ_NO_EXTI2		8		/* IRQ (Interrupt Request) number of EXTI2 */
#define IRQ_NO_EXTI3		9		/* IRQ (Interrupt Request) number of EXTI3 */
#define IRQ_NO_EXTI4		10		/* IRQ (Interrupt Request) number of EXTI4 */
#define IRQ_NO_EXTI9_5		23		/* IRQ (Interrupt Request) number of EXTI5-10 */
#define IRQ_NO_EXTI15_10	40		/* IRQ (Interrupt Request) number of EXTI10-15 */

#define IRQ_NO_SPI1			35		/* IRQ (Interrupt Request) number of SPI1 */
#define IRQ_NO_SPI2			36		/* IRQ (Interrupt Request) number of SPI2 */
#define IRQ_NO_SPI3			51		/* IRQ (Interrupt Request) number of SPI3 */

#define IRQ_NO_I2C1_EV		31		/* IRQ (Interrupt Request) number of I2C1_EV */
#define IRQ_NO_I2C1_ER		32		/* IRQ (Interrupt Request) number of I2C1_ER */
#define IRQ_NO_I2C2_EV		33		/* IRQ (Interrupt Request) number of I2C2_EV */
#define IRQ_NO_I2C2_ER		34		/* IRQ (Interrupt Request) number of I2C2_EV */
#define IRQ_NO_I2C3_EV		72		/* IRQ (Interrupt Request) number of I2C3_EV */
#define IRQ_NO_I2C3_ER		73		/* IRQ (Interrupt Request) number of I2C3_ER */

#define IRQ_NO_USART1		37		/* IRQ (Interrupt Request) number of USART1 */
#define IRQ_NO_USART2		38		/* IRQ (Interrupt Request) number of USART2 */
#define IRQ_NO_USART3		39		/* IRQ (Interrupt Request) number of USART3 */
#define IRQ_NO_UART4		52		/* IRQ (Interrupt Request) number of UART4 */
#define IRQ_NO_UART5		53		/* IRQ (Interrupt Request) number of UART5 */
#define IRQ_NO_USART6		71		/* IRQ (Interrupt Request) number of USART6 */
#define IRQ_NO_UART7				/* Not implemented in STM32F407xx - IRQ (Interrupt Request) number of UART7 */
#define IRQ_NO_UART8				/* Not implemented in STM32F407xx - IRQ (Interrupt Request) number of UART8 */

/*
 *	Macros for all possible priority levels
 */
#define NVIC_IRQ_PRI0		0		/* Highest priority level */
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15		/* Lowest priority level */

/*
 *	Some generic macros
 */

#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/*************************************************************************
 *
 * Bit position definitions of SPI peripheral
 *
 *************************************************************************/
/*
 *	Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 *	Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 *	Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/*************************************************************************
 *
 * Bit position definitions of I2C peripheral
 *
 *************************************************************************/
/*
 *	Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/*
 *	Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/*
 *	Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0		0
#define I2C_OAR1_ADD7_1		1
#define I2C_OAR1_ADD9_8		8
#define I2C_OAR1_ADDMODE	15

/*
 *	Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/*
 *	Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/*
 *	Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/*
 *	Trise(max) values of I2C
 */
#define I2C_TRISEMAX_SM		(1000 / 1000000000U)	// 1000 ns
#define I2C_TRISEMAX_FM		(300 / 1000000000U)		// 300 ns

/*************************************************************************
 *
 * Bit position definitions of USART peripheral
 *
 *************************************************************************/
/*
 *	Bit position definitions USART_SR
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE 		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

/*
 *	Bit position definitions USART_BRR
 */
#define USART_BRR_DIV_FRACTION		0
#define USART_BRR_DIV_MANTISSA		4

/*
 *	Bit position definitions USART_CR1
 */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

/*
 *	Bit position definitions USART_CR2
 */
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

/*
 *	Bit position definitions USART_CR3
 */
#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
