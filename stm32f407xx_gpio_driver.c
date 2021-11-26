/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 7, 2021
 *      Author: Satyam Kumar
 */

#include "stm32f407xx_gpio_driver.h"


/*
 * Peripheral CLock Setup
 */
/************************************************************
 * @FunctionName 		:	GPIO_PeriClockControl
 * @brief				:	enable/Disable the clock of GPIOx peripheral
 *
 * @param[in]			:	GPIO_RegDef_t *pGPIOx
 * @param[in]			:	uint8_t EnorDi
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
void GPIO_PeriControl_Clock(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		//Enable the clock
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else{

		//Disable the clock
		if(pGPIOx == GPIOA){
					GPIOA_PCLK_DI();
				}else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}
}


/*
 * Init and Dein Init
 */
/************************************************************
 * @FunctionName 		:	GPIO_Init
 * @brief				:	Initializing the GPIO Peripheral
 *
 * @param[in]			:	GPIO_Handle_t *pGPIOHandle
 * @param[in]			:
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;    //temp register
	//1. Enable the Peripheral Clock
	GPIO_PeriControl_Clock(pGPIOHandle->pGPIOx, ENABLE);
	//2. C0nfigure the mode of the GPIO pin


	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
	pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber));
	pGPIOHandle->pGPIOx->MODER |= temp;

	temp = 0;
	//3. Configure the speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//4. Configure the PuPD Settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
	//5. Configure the optype
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//6. Configure the Alternate function Mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber / 8 ;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNUmber % 8 ;
		pGPIOHandle->pGPIOx->AF[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AF[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}


}
/************************************************************
 * @FunctionName 		:	GPIO_DeInit
 * @brief				:	Resetting all the registers of GPIOx peripheral
 *
 * @param[in]			:	GPIO_RegDef_t *pGPIOx
 * @param[in]			:
 *
 * @return				:	Null
 *
 * @Note				:
 ************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_REG_RESET();
		}
}

/*
 * Read and write operations
 */
/************************************************************
 * @FunctionName 		:	GPIO_ReadFromInputPin
 * @brief				:	Read the data from the input pin
 *
 * @param[in]			:	GPIO_RegDef_t *pGPIOx
 * @param[in]			:	uint8_t PinNumber
 *
 * @return				:	uint8_t
 *
 * @Note				:
 ************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber)&0x00000001) ;

	return value;

}
/************************************************************
 * @FunctionName 		:	GPIO_ReadFromInputPort
 * @brief				:	Reading the entire port
 *
 * @param[in]			:	GPIO_RegDef_t *pGPIOx
 * @param[in]			:
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}
/************************************************************
 * @FunctionName 		:	GPIO_WriteToInputPin
 * @brief				:	Writes to a particullar pin
 *
 * @param[in]			:	GPIO_RegDef_t *pGPIOx
 * @param[in]			:	uint8_t PinNumber
 * @param[in]			:	uint8_t value
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){

	if(value == GPIO_PIN_SET){
		//Write 1 to the ODR at the field corresponding to the pin number
		pGPIOx->ODR |= (1<<PinNumber);
	}else{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}
/************************************************************
 * @FunctionName 		:	GPIO_WriteToOutputPort
 * @brief				:	Writes to a the entire port
 *
 * @param[in]			:	GPIO_RegDef_t *pGPIOx
 * @param[in]			:	uint16_t value
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
		pGPIOx->ODR = value;
}
/************************************************************
 * @FunctionName 		:	GPIO_ToggleOutputPin
 * @brief				:	Toggles the particullar pin number
 *
 * @param[in]			:	GPIO_RegDef_t *pGPIOx
 * @param[in]			:	uint8_t PinNumber
 *
 * @return				:	None
 *
 * @Note				:
 ************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}
