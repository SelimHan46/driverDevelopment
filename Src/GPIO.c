
#include "GPIO.h"

/**
  * @brief  GPIO_Init, Configures the port and pin
  *
  * @param  GPIOx = GPIO Port Base Address
  *
  * @param  GPIO_InitTypeDef_t = User Config Structure, GPIO_Init
  *
  * @retval None (void)
  *
  */

void  GPIO_Init(GPIO_TypeDef_t  *GPIOx, GPIO_InitTypeDef_t *GPIO_Init)
{
	uint32_t position;
	uint32_t fakePosition = 0;
	uint32_t lastPosition = 0;

	for(position = 0; position < 16; position++)
	{

		fakePosition = (0x1 << position);
		lastPosition = (uint32_t)(GPIO_Init->Pin) & fakePosition;

		if(fakePosition == lastPosition)
		{

			/* Mode Config*/

			uint32_t tempValue = GPIOx->MODER;

			tempValue &= ~(0x3U << (position * 2));
			tempValue |= (GPIO_Init->Mode << (position * 2));

			GPIOx->MODER = tempValue;

			if(GPIO_Init->Mode == GPIO_MODE_OUTPUT ||GPIO_Init->Mode ==  GPIO_MODE_AF)
			{

				/* Output Type CONFİG */
				tempValue = GPIOx->OTYPER;
				tempValue &= ~(0x1U << position);
				tempValue |= (GPIO_Init->Otype << position);
				GPIOx->OTYPER = tempValue;

			}

			/* Output Speed CONFİG */

			 tempValue = GPIOx->OSPEEDR;

			tempValue &= ~(0x3U << (position * 2));
			tempValue |= (GPIO_Init->Speed << (position * 2));

			GPIOx->OSPEEDR = tempValue;

			/* PULL CONFİG */

			 tempValue = GPIOx->PUPDR;

			tempValue &= ~(0x3U << (position * 2));
			tempValue |= (GPIO_Init->Pull << (position * 2));

			GPIOx->PUPDR = tempValue;

		}

	}
}

/**
  * @brief  GPIO_Write_Pin, makes pin High or Low
  *
  * @param  GPIOx = GPIO Port Base Address
  *
  * @param  GPIO_Pin = GPIO_Pin_Number 0 -15:
  *
  * @param pinState = GPIO_Pin_Set OR GPIO_Pin_RESET
  *
  * @retval None (void)
  */

void GPIO_WritePin(GPIO_TypeDef_t* GPIOx, uint16_t GPIO_Pin, GPIO_PinState_t PinState)
{

	  if(PinState != GPIO_PIN_RESET)
	  {
	    GPIOx->BSRR = GPIO_Pin;
	  }
	  else
	  {
	    GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
	  }

}

/**
  * @brief  GPIO_Read_Pin, read the pin of GPIOx Port
  *
  * @param  GPIOx = GPIO Port Base Address
  *
  * @param  GPIO_Pin = GPIO_Pin_Number 0 -15:
  *
  * @retval GPIO_PinState_t
  */

GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_PinState_t bitstatus;

	  if((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
	  {
	    bitstatus = GPIO_PIN_SET;
	  }
	  else
	  {
	    bitstatus = GPIO_PIN_RESET;
	  }
	  return bitstatus;
}

/**
  * @brief  GPIO_Lock_Pin, locks the pin of GPIOx Port
  *
  * @param  GPIOx = GPIO Port Base Address
  *
  * @param  GPIO_Pin = GPIO_Pin_Number 0 -15:
  *
  * @retval void
  */

void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, uint16_t GPIO_Pin)
{
	uint32_t tempValue = (0x1U << 16U) | GPIO_Pin;

	GPIOx->LCKR = tempValue; // LCKR[16] = '1' LCKR =[15:0] = DATA
	GPIOx->LCKR = GPIO_Pin;  // LCKR[16] = '0' LCKR =[15:0] = DATA
	GPIOx->LCKR = tempValue; // LCKR[16] = '1' LCKR =[15:0] = DATA
	tempValue = GPIOx->LCKR; // Read Lock Register

}

/**
  * @brief  GPIO_Toggle_Pin, locks the pin of GPIOx Port
  *
  * @param  GPIOx = GPIO Port Base Address
  *
  * @param  GPIO_Pin = GPIO_Pin_Number 0 -15:
  *
  * @retval void
  */

void GPIO_TogglePin(GPIO_TypeDef_t* GPIOx, uint16_t GPIO_Pin)
{
	uint32_t tempODRRegister = GPIOx->ODR;
	GPIOx->BSRR = ( ( tempODRRegister & GPIO_Pin ) << 16U ) | ( ~tempODRRegister & GPIO_Pin);
}
