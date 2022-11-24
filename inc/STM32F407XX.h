/*
 * STM32F407XX.h
 *
 *  Created on: Oct 7, 2022
 *      Author: ahuja
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*Header files*/
#include<stdint.h>

/*Additional Macros*/
#define __vo volatile //volatile

/*Processor specific */
#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)

#define NVIC_IPR			((__vo uint32_t*)0xE000E400) //priority
#define NO_OF_PRIORITY_SHIFTS	4


/*Memory registers*/
#define FLASH_BASEADDR		0x08000000U	 	//flash address
#define SRAM1_BASEADDR		0x20000000U		//sram1 address
#define SRAM2_BASEADDR		0x20001C00U		//sram2 address
#define ROM_BASEADDR		0x1FFF0000U		//rom address/ system memory
#define SRAM				SRAM1_BASEADDR	//sram1 is sram

/*Bus base register*/
#define PERIBASE			0x40000000U
#define APB1_BASEADDR		PERIBASE		//apb1 bus
#define APB2_BASEADDR		0x40010000U		//apb2 bus
#define AHB1_BASEADDR		0x40020000U		//ahb1 bus
#define AHB2_BASEADDR		0x50000000U		//ahb2 bus

/*Base address of peripheral hanging on AHB1*/
#define GPIOA_BASEADDR		(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR		(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1_BASEADDR + 0x2000)
#define RCC_BASEADDR		(AHB1_BASEADDR + 0X3800)


/*Base address of peripheral hanging on APB1*/
#define I2C1_BASEADDR		(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR		(APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR		(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1_BASEADDR + 0x4800)

#define UART4_BASEADDR		(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1_BASEADDR + 0x5000)

/*Base address of peripheral hanging on APB2*/
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00)

#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR		(APB2_BASEADDR + 0x3400)

#define USART1_BASEADDR		(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400)

#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800)




/*Peripheral register structures for GPIO*/
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIORegDef_t;

/*
 * Clock register definition
 */
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

/*
 * EXTI (interrupt) register definition
 */
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

/*
 * SYSCFG(interrupt) register definition
 */
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED[2]; //Reserved because after 0x14 , the next register start from 0x20. Hence the gap is 8
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;


/*
 * SPI Registers
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t TRISE;
	__vo uint32_t CCR;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

/*Peripheral definitions*/
#define GPIOA				((GPIORegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIORegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIORegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIORegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIORegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIORegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIORegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIORegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIORegDef_t*)GPIOI_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * SPI Peripheral
 */
#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * I2C Peripheral
 */
#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)


/*Clock enable macros for GPIOx peripheral*/

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1<<8))

/*Clock enable macros for I2C peripheral*/

#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1<<23))


/*Clock enable macros for SPI peripheral*/

#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()	(RCC->APB2ENR |= (1<<13))

/*Clock enable macros for UART peripheral*/

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1<<5))

/*Clock enable macros for SYSCFG peripheral*/

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1<14))


/*Clock disable macros for GPIOx peripheral*/

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1<<8))

/*Clock disable macros for I2C peripheral*/

#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~(1<<23))


/*Clock disable macros for SPI peripheral*/

#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()	(RCC->APB2ENR &= ~(1<<13))

/*Clock disable macros for UART peripheral*/

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1<<5))

/*Clock disable macros for SYSCFG peripheral*/

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1<14))

/*Resetting the GPIO registers*/
#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<0 )); (RCC->AHB1RSTR &= ~( 1<<0 )); }while(0);
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<1 )); (RCC->AHB1RSTR &= ~( 1<<1 )); }while(0);
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<2 )); (RCC->AHB1RSTR &= ~( 1<<2 )); }while(0);
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<3 )); (RCC->AHB1RSTR &= ~( 1<<3 )); }while(0);
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<4 )); (RCC->AHB1RSTR &= ~( 1<<4 )); }while(0);
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<5 )); (RCC->AHB1RSTR &= ~( 1<<5 )); }while(0);
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<6 )); (RCC->AHB1RSTR &= ~( 1<<6 )); }while(0);
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<7 )); (RCC->AHB1RSTR &= ~( 1<<7 )); }while(0);
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= ( 1<<8 )); (RCC->AHB1RSTR &= ~( 1<<8 )); }while(0);


/*
 * Resetting SPI registers
 */

#define SPI1_REG_RESET()	do{(RCC->APB2RSTR |= ( 1<<12 )); (RCC->APB2RSTR &= ~( 1<<12 ));}while(0);
#define SPI2_REG_RESET()	do{(RCC->APB1RSTR |= ( 1<<14 )); (RCC->APB1RSTR &= ~( 1<<14 ));}while(0);
#define SPI3_REG_RESET()	do{(RCC->APB1RSTR |= ( 1<<15 )); (RCC->APB1RSTR &= ~( 1<<15 ));}while(0);
#define SPI4_REG_RESET()	do{(RCC->APB2RSTR |= ( 1<<13 )); (RCC->APB2RSTR &= ~( 1<<13 ));}while(0);


//GPIO Baseaddr to code(portcode)
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0:\
									(x == GPIOB) ? 1:\
									(x == GPIOC) ? 2:\
									(x == GPIOD) ? 3:\
									(x == GPIOE) ? 4:\
									(x == GPIOF) ? 5:\
									(x == GPIOG) ? 6:\
									(x == GPIOH) ? 7:\
									(x == GPIOI) ? 8:0 )

//IRQ CONFIG SETTING ON PROCESSOR SIDE
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI5_9		23
#define IRQ_NO_EXTI10_15	40



/*Some generic Macros*/
#define ENABLE 				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		ENABLE
#define GPIO_PIN_RESET		DISABLE
#define FlagReset			RESET
#define FlagSet				SET

/*
 * Bit Definition macros for SPI peripheral
 */
//spi_cr1
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


//spi_cr2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

//Spi_sr
#define SPI_SR_RXNIE		0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8



/*
 * I2C
 */
//i2c_cr1
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK 		10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

//i2c_cr2
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12


//i2c_oar1
#define I2C_OAR1_ADDO		0
#define I2C_OAR1_ADD[7:1]	1
#define I2C_OAR1_ADD[9:8]	8
#define I2C_OAR1_ADDMODE	15

//i2c_oar2
#define I2C_OAR2_ENDUAL		0
#define I2C_OAR2_ADD2[7:1]	1

//i2c_sr1
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF 		2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF 			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

//i2c_sr2
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT 	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

//i2c_ccr
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_F/S			15


/*
 * Resetting I2C
 */

#define I2C1_REG_RESET()			do{(RCC->APB1RSTR |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21));}while(0);
#define I2C2_REG_RESET()			do{(RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22));}while(0);
#define I2C3_REG_RESET()			do{(RCC->APB!RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23));}while(0);



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#endif /* INC_STM32F407XX_H_ */
