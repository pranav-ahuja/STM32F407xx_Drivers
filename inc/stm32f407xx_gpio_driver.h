/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 14, 2022
 *      Author: ahuja
 */
#include "STM32F407XX.h"

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

/*Structure for GPIO Pins*/
typedef struct{
	uint8_t GPIO_PinNumber;				/* Possible value from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;				/* Possible value from @GPIO_PIN_MODE*/
	uint8_t GPIO_PinSpeed;				/* Possible value from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*Structure to handle GPIO Pin*/
typedef struct
{
	//pointer to hold the base address of GPIO
	GPIORegDef_t *pGPIOx;				/*Base address of */
	GPIO_PinConfig_t GPIO_pinconfig;
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBER
 * GPIO Pin Number
 */
#define GPIO_PIN_NO_0	0
#define GPIO_PIN_NO_1	1
#define GPIO_PIN_NO_2	2
#define GPIO_PIN_NO_3	3
#define GPIO_PIN_NO_4	4
#define GPIO_PIN_NO_5	5
#define GPIO_PIN_NO_6	6
#define GPIO_PIN_NO_7	7
#define GPIO_PIN_NO_8	8
#define GPIO_PIN_NO_9	9
#define GPIO_PIN_NO_10	10
#define GPIO_PIN_NO_11	11
#define GPIO_PIN_NO_12	12
#define GPIO_PIN_NO_13	13
#define GPIO_PIN_NO_14	14
#define GPIO_PIN_NO_15	15


/*
 * @GPIO_PIN_MODE
 * GPIO Modes values
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * GPIO Output type
 */
#define GPIO_OPTYPE_PP		0
#define GPIO_OPTYPE_OD		1

/*
 * @GPIO_PIN_SPEED
 * GPIO Output speed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VHIGH	3

/*
 * GPIO Pullup/Pulldown
 */
#define GPIO_NOPUPUD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2



/*******************API support for GPIO pins************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle); 												//GPIO Initialisation
void GPIO_DeInit(GPIORegDef_t *pGPIOx);														//GPIO Deinitialisation
void GPIO_PeriClockControl(GPIORegDef_t *pGPIOx, uint8_t ENorDi );  						//Peripheral clock control
uint8_t GPIO_ReadFromInputPin(GPIORegDef_t *pGPIOx, uint8_t PinNumber);						//Reading from input pin
uint16_t GPIO_ReadFromInputPort(GPIORegDef_t *pGPIOx);										//Reading from input port
void GPIO_WriteToOutputPin(GPIORegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);			//writing to output pin
void GPIO_WriteToOutputPort(GPIORegDef_t *pGPIOx, uint16_t Value);							//writing to output port
void GPIO_ToggleOutputPin(GPIORegDef_t *pGPIOx, uint8_t PinNumber);							//Toggle output pin
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);						 				//used to config the interrupt registers
void GPIO_IRQHandling(uint8_t PinNumber); 													//used to handle the interrupt
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t IRQPriority);								//Used to set IRQ priority


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
