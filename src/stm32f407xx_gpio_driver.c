/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 14, 2022
 *      Author: ahuja
 */


#include "stm32f407xx_gpio_driver.h"

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

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) 												//GPIO Initialisation
{

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);


	uint32_t temp = 0;
	if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		/*
		 * Moder register
		 */
		temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber));		//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else {
		if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
			//clear RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
			//clear FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
		}
		//interrupt, code later

		//2. Config the GPIO port selection in SYSCONFIG
		uint8_t temp1, temp2, portcode;
		temp1 = pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber % 4;
		portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);


		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4) );


		//3. Enable EXTI interrupt delivery using IMR.
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
	}

	temp = 0;

			/*
			 * Speed register
			 */
			temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber));		//clearing
			pGPIOHandle->pGPIOx->OSPEEDR |= temp;
			temp = 0;

			/*
			 * Pin Output Type
			 */
			temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinOPType << pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->OTYPER &= ~(1 <<  pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber);		//clearing
			pGPIOHandle->pGPIOx->OTYPER |= temp;
			temp = 0;

			/*
			 * Pin Pull up/Pull down
			 */
			temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber));		//clearing
			pGPIOHandle->pGPIOx->PUPDR |= temp;
			temp = 0;

			/*
			 * Alternate Function
			 */
			if(pGPIOHandle->GPIO_pinconfig.GPIO_PinMode == GPIO_MODE_ALTFN){
//				if(pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber <= 7) //Alternate function low
//				{
//					temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber));
//					pGPIOHandle->pGPIOx->AFR[0] |= temp;
//					temp = 0;
//				}
//				else { //Alternate function high
//					temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber - 8)));
//					pGPIOHandle->pGPIOx->AFR[1] |= temp;
//					temp = 0;
//				}

				/*************************************************OR*********************************************************************/
				uint32_t temp1, temp2;
				temp1 = pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber / 8;
				temp2 = pGPIOHandle->GPIO_pinconfig.GPIO_PinNumber % 8;

				temp = (pGPIOHandle->GPIO_pinconfig.GPIO_PinAltFunMode << (4 * temp2));
				pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));		//clearing
				pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
				temp = 0;
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

void GPIO_DeInit(GPIORegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}

			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}

			else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}

			else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}

			else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}

			else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}

			else if(pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}

			else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}

			else if(pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
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

void GPIO_PeriClockControl(GPIORegDef_t *pGPIOx, uint8_t ENorDi )
{
	if(ENorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}

		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}

		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}

		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}

		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}

		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}

		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}

		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
				if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}

				else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}

				else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}

				else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}

				else if(pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}

				else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}

				else if(pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}

				else if(pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}

				else if(pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
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


uint8_t GPIO_ReadFromInputPin(GPIORegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
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


uint16_t GPIO_ReadFromInputPort(GPIORegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
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


void GPIO_WriteToOutputPin(GPIORegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to bit field
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
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

void GPIO_WriteToOutputPort(GPIORegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
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
void GPIO_ToggleOutputPin(GPIORegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
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
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){
		//used to config the interrupt registers
	//coding at processor side so use Cortex m4 user guide
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else{
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
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

void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t section = IRQNumber % 4;

	uint8_t shiftamount = (section * 8) + (8 - NO_OF_PRIORITY_SHIFTS);
	*(NVIC_IPR + (iprx*4)) |= (IRQPriority << (shiftamount));
}

void GPIO_IRQHandling(uint8_t PinNumber){

}
