/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 15, 2022
 *      Author: ahuja
 */

#include "stm32f407xx_spi_driver.h"

/******************************************************************************
 * @fn				:
 *
 * @brief			:
 *
 * @param[in]		:
 * @param[in]		:
 * @param[in]		:
 *
 * @return			:
 *
 * @Note			:
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//SPI Initialisation
	//1. Comfig SPI_CR1 register

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t temp = 0;

	//1.a. Select Device mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR; //device mode

	//1.b. Select Bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) //bus config | full duplex
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE); //bidimode
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) //half duplex
	{
		temp |= (1 << SPI_CR1_BIDIMODE); //bidimode
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE); // bidimode
		temp |= (1 << SPI_CR1_RXONLY); //rx only
	}

	//1.c. Clock Speed
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//1.d. DFF
	temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//1.e CPOL
	temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//1.f. CPHA
	temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//Put value in SPI_CR1
	pSPIHandle->pSPIx->CR1 = temp;

}

/******************************************************************************
 * @fn				:
 *
 * @brief			:
 *
 * @param[in]		:
 * @param[in]		:
 * @param[in]		:
 *
 * @return			:
 *
 * @Note			:
 *
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	//SPI De-initialisation
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/******************************************************************************
 * @fn				:
 *
 * @brief			:
 *
 * @param[in]		:
 * @param[in]		:
 * @param[in]		:
 *
 * @return			:
 *
 * @Note			:
 *
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI ){
	//Peripheral clock control
	if(ENorDI == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}

			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}

			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}

			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}

			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}

			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}

/*
 * DATA SEND AND RECEIVE
 */
//Polling way
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	//Sending data

	while(Len > 0){
		//if flag is set
		while((SPIGetFlagStatus(pSPIx, SPI_TXE_FLAG)) == FlagReset);
			//Check for DFF
			if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
				//16bit
				//load the data in DR
				pSPIx->DR = *((uint16_t*)pTxBuffer);
				Len-=2;
				(uint16_t*)pTxBuffer++;
			}
			else{
				//8 bit
				//load data to DFF
				pSPIx->DR = *pTxBuffer;
				Len--;
				pTxBuffer++;
			}
		}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)			//Receive Spi Data
{

	while(Len>0){
		while(SPIGetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FlagReset);

		if(pSPIx->CR1 & ( 1<<SPI_CR1_DFF)){
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}

}

void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI == ENABLE){
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );
	}
	else if(ENorDI == DISABLE)
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
	}
}


uint8_t SPIGetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName){
		return FlagSet;
	}

	return FlagReset;
}


void SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE){
			pSPIx->CR1 |= ( 1 << SPI_CR1_SSI );
		}
		else if(ENorDI == DISABLE)
		{
			pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI );
		}
}
