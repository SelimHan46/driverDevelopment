#include "SPI.h"

/**
  * @brief  SPI_Init, Configures the SPI Peripheral
  *
  * @param  SPI_Handle =  User config structure
  *
  * @retval None (void)
  */


void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle)
{
	uint32_t tempValue = 0;

	tempValue = SPI_Handle->Instance->CR1;

	tempValue |= (SPI_Handle->Init.SPI_BaudRate) | (SPI_Handle->Init.SPI_CPHA) | (SPI_Handle->Init.SPI_CPOL) | (SPI_Handle->Init.DFF_Format) |
			     (SPI_Handle->Init.SPI_Mode) | (SPI_Handle->Init.FrameFormat) | (SPI_Handle->Init.BusConfig) | (SPI_Handle->Init.SSM_Cmd);

	SPI_Handle->Instance->CR1 = tempValue;
}

/**
  * @brief  SPI_PeriphCmd, Enable or Disable SPI Peripheral
  *
  * @param  SPI_Handle = User config structure
  *
  * @param  stateOfSPI = ENABLE or DISABLE
  *
  * @retval None (void)
  *
  */

void SPI_PeriphCmd(SPI_HandleTypeDef_t *SPI_Handle, FunctionalState_t stateOfSPI)
{
	if( stateOfSPI == ENABLE )
	{
		SPI_Handle->Instance->CR1 |= (0x1U << SPI_CR1_SPE);
	}
	else
	{
		SPI_Handle->Instance->CR1 &= ~(0x1U << SPI_CR1_SPE);
	}

}
