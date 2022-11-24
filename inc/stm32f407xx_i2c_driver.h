/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: Nov 23, 2022
 *      Author: ahuja
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "STM32F407XX.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t	*pI2Cx;
	I2C_Config_t	I2C_Config;
}I2C_Handle_t;


/*
 * Speed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM2K	200000
#define I2C_SCL_SPEED_FM4K	400000

/*
 * ACK Control
 */
#define I2C_ACK_EN			1
#define I2C_ACK_DI			0


/*
 * I2C Duty Cycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1



/***********************************************************api***************************************************/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
void I2C_MasterTx();

//Other peripherals
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
uint8_t I2CGetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
