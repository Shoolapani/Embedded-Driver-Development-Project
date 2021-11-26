/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 7, 2021
 *      Author: Satyam Kumar
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * GPIO_PinNumber
 */
#define GPIO_PIN_NO_0 			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15
/*
 * GPIO_PinMode
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3



/*
 * GPIO_PinSpeed
 */
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST 		2
#define GPIO_SPEED_HIGH 		3

/*
 * GPIO_PinOpType
 */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1


/*
 * GPIO_PinPuPdControl
 */

#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


/*
 * This is pin Configuration Structure for the GPIO pins
 */


typedef struct{

	uint8_t GPIO_PinNUmber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * Handle structure for GPIO
 *
 */

typedef struct{

	GPIO_RegDef_t *pGPIOx;

	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;

/********************************************************
 * APIs Sopported by the driver
 ********************************************************/

/*
 * Peripheral CLock Setup
 */

void GPIO_PeriControl_Clock(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and Dein Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and write operations
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToInputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
