/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Nov 15, 2022
 *      Author: ahuja
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "STM32F407XX.h"

/*
 * Structure for SPI config items
 */
typedef struct
{
	uint8_t SPI_DeviceMode; //master or slave
	uint8_t SPI_BusConfig;  //full or half duplex
	uint8_t SPI_SclkSpeed;	//clock speed
	uint8_t SPI_DFF; 		//8bit or 16bit
	uint8_t SPI_CPHA;		//clock
	uint8_t SPI_CPOL;		//clock
	uint8_t SSM;			//slave select
	uint8_t Speed;			//speed of data
}SPI_Config_t;

/*
 * SPI Handle Structure
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;	//handle registers
	SPI_Config_t SPIConfig;	//handle config items
}SPI_Handle_t;


/*
 * @SPI_DeviceMode || Reg - CR1(MSTR)
 */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0


/*
 * @SPIBusConfig  || Reg - CR!(BIDIOE & BIDIMODE & RXONLY(Simplex, receive only mode))
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_S_RXONLY				3

/*
 * @SPIDFF
 */
#define SPI_DFF_8							0
#define	SPI_DFF_16							1


/*
 *  @SPIClockSpeed || Reg - CR1(BR)
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
 * @cpol  || Reg = CR1(CPOL)
 */
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

/*
 * @CPHA || Reg = CR1(CPHA)
 */
#define	SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

/*
 * @SSM
 */
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

//TXE flag
#define SPI_TXE_FLAG						( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG						( 1 << SPI_SR_RXNIE)

/*
 * iNITIALISATION OF SPI AND CLOCK
 */
void SPI_Init(SPI_Handle_t *pSPIHandle); 												//SPI Initialization
void SPI_DeInit(SPI_RegDef_t *pGPIOx);													//SPI De-initialisation
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDi );  						//Peripheral clock control

/*
 * DATA SEND AND RECEIVE
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
/*
 * INTERRUPT
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi); 			//used to config the interrupt registers
void SPI_IRQHandling(SPI_Handle_t *pHandle); 											//used to handle the interrupt
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t IRQPriority);							//Used to set IRQ priority

/*
 * OTHER PERIPHERAL CONTROL APIs
 */
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
uint8_t SPIGetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDi);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
