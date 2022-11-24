/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Nov 23, 2022
 *      Author: ahuja
 */

#include "stm32f407xx_i2c_driver.h"

uint16_t AHBPrescaler = {2,4,8,16,32,64,128,256,512};
uint8_t APB1Prescaler = {2,4,8,16};

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI){
	if(ENorDI == ENABLE)
	{
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else if(ENorDI == DISABLE)
	{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}


void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint8_t temp = 0;

	temp |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);
	pI2CHandle->pI2Cx->CR1 = temp;

	//configure freq
	temp = 0;
	temp |= RCC_PCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (temp & 0x3F);

	//Storing Address in OAR
	temp = 0;
	temp |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	temp |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = temp;


	//Clock control register
	uint16_t CCRValue = 0;
	temp = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		CCRValue = RCC_PCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp |= CCRValue & 0xfff;
	}
	else{
		temp |= (1<<15);//fast mode
		temp |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			CCRValue = RCC_PCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			temp |= CCRValue & 0xfff;
		}
		else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9){
			CCRValue = RCC_PCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			temp |= CCRValue & 0xfff;
		}
	}

	pI2CHandle->pI2Cx->CCR |= temp;





}

uint32_t RCC_PCLK1Value(void)
{
	uint8_t clksrc, ahbp, apbp1;
	uint16_t prescale, preapb1;
	uint32_t pclk, SystemClk;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}

	//AHB
	prescale = (RCC->CFGR >> 4) & 0xF;

	if(prescale < 8)
	{
		ahbp=1;
	}
	else{
		ahbp = AHBPrescaler[prescale - 8];
	}

	preapb1 = (RCC->CFGR >> 10) & 0x7;

	//APB1
	if(preapb1 < 4)
	{
		apbp1 = 1;
	}
	else{
		apbp1 = APB1Prescaler[preapb1 - 4];
	}

	pclk = (SystemClk / ahbp) / apbp1;

	return pclk;

}



void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3){
		I2C3_PCLK_EN();
	}
}
