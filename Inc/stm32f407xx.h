/*
 * stm32f407xx.h
 *
 *  Created on: Jun 11, 2023
 *      Author: 90543
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <string.h>

/*
 * Microprocessor Defines
 * */

#define NVIC_ISER0				( (uint32_t*)(0xE000E100UL) )


#define __IO	volatile

#define SET_BIT(REG,BIT)		   ( (REG) |= (BIT)  )
#define CLEAR_BIT(REG,BIT)		   ( (REG) &= ~(BIT) )
#define READ_BIT(REG,BIT)		   ( (REG)  &  (BIT) )
#define UNUSED(x)			       (void)x

typedef enum
{
	DISABLE = 0x0U,
	ENABLE = !DISABLE,

}FunctionalState_t;


/*
 * IRQ Numbers of MCU ==  Vector Table
 * */

typedef enum
{
	EXTI0_IRQNumber = 6,
	EXTI1_IRQNumber = 7,
	EXTI2_IRQNumber = 8,
	EXTI3_IRQNumber = 9,

}IRQNumber_TypeDef_t;



/*
 * Memory Base Address
 */

#define FLASH_BASE_ADDR				(0x08000000UL)		/* Flash Base Address (up to 1MB) */
#define SRAM1_BASE_ADDR 			(0x20000000UL)		/* SRAM1 Base Address 112KB       */
#define SRAM2_BASE_ADDR             (0x2001C000UL)		/* SRAM2 Base Address 16KB        */


/*
 * Peripheral Base Addresses
 */

#define PERIPH_BASE_ADDR			(0x40000000UL)						/* Base Address for All Peripherals */
#define APB1_BASE_ADDR              PERIPH_BASE_ADDR					/* APB1 Bus Domain Base Address     */
#define APB2_BASE_ADDR              (PERIPH_BASE_ADDR + 0x00010000UL)	/* APB2 Bus Domain Base Address     */
#define AHB1_BASE_ADDR       		(PERIPH_BASE_ADDR + 0x00020000UL)	/* AHB1 Bus Domain Base Address     */
#define AHB2_BASE_ADDR       		(PERIPH_BASE_ADDR + 0x10000000UL)	/* AHB2 Bus Domain Base Address     */

/*
 * APB1 Peripherals Base Addresses
 */

#define TIM2_BASE_ADDR 			(APB1_BASE_ADDR + 0x0000UL)		/* Timer 2 Base Address */
#define TIM3_BASE_ADDR 			(APB1_BASE_ADDR + 0x0400UL)		/* Timer 3 Base Address */
#define TIM4_BASE_ADDR 			(APB1_BASE_ADDR + 0x0800UL)		/* Timer 4 Base Address */

#define SPI2_BASE_ADDR			(APB1_BASE_ADDR + 0x3800UL)		/* SPI 2 Base Address   */
#define SPI3_BASE_ADDR 			(APB1_BASE_ADDR + 0x3C00UL)		/* SPI 3 Base Address   */

#define USART2_BASE_ADDR        (APB1_BASE_ADDR + 0x4400UL)		 /* USART 2 Base Address */
#define USART3_BASE_ADDR        (APB1_BASE_ADDR + 0x4800UL)	     /* USART 3 Base Address */
#define UART4_BASE_ADRR         (APB1_BASE_ADDR + 0x4C00UL)      /* UART 4 Base Address  */
#define UART5_BASE_ADRR         (APB1_BASE_ADDR + 0x5000UL)      /* UART 5 Base Address  */

#define I2C1_BASE_ADDR          (APB1_BASE_ADDR + 0x5400UL)      /* I2C 1 Base Address   */
#define I2C2_BASE_ADDR          (APB1_BASE_ADDR + 0x5800UL)	     /* I2C 2 Base Address   */
#define I2C3_BASE_ADRR          (APB1_BASE_ADDR + 0x5C00UL)      /* I2C 3 Base Address   */

/*
 * APB2 Peripherals Base Addresses
 */

#define TIM1_BASE_ADDR          (APB2_BASE_ADDR + 0x0000UL)	     /* Timer 1 Base Address */
#define TIM8_BASE_ADDR          (APB2_BASE_ADDR + 0x0400UL)      /* Timer 8 Base Address */

#define USART1_BASE_ADDR 	    (APB2_BASE_ADDR + 0x1000UL)      /* USART 1 Base Address */
#define USART6_BASE_ADRR        (APB2_BASE_ADDR + 0x1400UL)      /* USART 6 Base Address */

#define SPI1_BASE_ADRR          (APB2_BASE_ADDR + 0x3000UL)      /* SPI 1 Base Address   */
#define SPI4_BASE_ADRR          (APB2_BASE_ADDR + 0x3000UL)      /* SPI 4 Base Address   */
#define SYSCFG_BASE_ADDR        (APB2_BASE_ADDR + 0x3800UL)      /* SYSCFG Base Address  */
#define EXTI_BASE_ADDR          (APB2_BASE_ADDR + 0x3C00UL)      /* EXTI Base Address    */


/*
 * AHB2 Peripherals Base Addresses
 */

#define GPIOA_BASE_ADDR         (AHB1_BASE_ADDR + 0x0000UL)     /* GPIOA Base Address    */
#define GPIOB_BASE_ADDR         (AHB1_BASE_ADDR + 0x0400UL)     /* GPIOB Base Address    */
#define GPIOC_BASE_ADDR         (AHB1_BASE_ADDR + 0x0800UL)     /* GPIOC Base Address    */
#define GPIOD_BASE_ADDR         (AHB1_BASE_ADDR + 0x0C00UL)     /* GPIOD Base Address    */
#define GPIOE_BASE_ADDR         (AHB1_BASE_ADDR + 0x1000UL)     /* GPIOE Base Address    */

#define RCC_BASE_ADDR           (AHB1_BASE_ADDR + 0x3800UL)     /* RCC Base Address      */

/*
 * Peripheral Structure Definitions
 */

typedef struct
{
  __IO uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  __IO uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef_t;

typedef struct
{
  __IO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __IO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __IO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __IO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __IO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_TypeDef_t;

/**
  * @brief System configuration controller
  */

typedef struct
{
  __IO uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __IO uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  __IO uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  __IO uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef_t;

/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __IO uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  __IO uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __IO uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __IO uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __IO uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef_t;

/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  __IO uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  __IO uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  __IO uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  __IO uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  __IO uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  __IO uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  __IO uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef_t;

#define GPIOA				( (GPIO_TypeDef_t *)(GPIOA_BASE_ADDR) )
#define GPIOB				( (GPIO_TypeDef_t *)(GPIOB_BASE_ADDR) )
#define GPIOC				( (GPIO_TypeDef_t *)(GPIOC_BASE_ADDR) )
#define GPIOD				( (GPIO_TypeDef_t *)(GPIOD_BASE_ADDR) )
#define GPIOE				( (GPIO_TypeDef_t *)(GPIOE_BASE_ADDR) )

#define RCC					( (RCC_TypeDef_t *)(RCC_BASE_ADDR)    )

#define SYSCFG              ( (SYSCFG_TypeDef_t *)(SYSCFG_BASE_ADDR) )

#define EXTI			    ( (EXTI_TypeDef_t *)(EXTI_BASE_ADDR) )

#define SPI1 				( (SPI_TypeDef_t)(SPI1_BASE_ADRR) )
#define SPI2 				( (SPI_TypeDef_t)(SPI2_BASE_ADRR) )
#define SPI3 				( (SPI_TypeDef_t)(SPI3_BASE_ADRR) )
#define SPI4 				( (SPI_TypeDef_t)(SPI4_BASE_ADRR) )

/*
 * Bit Definitions
 */

#define RCC_AHB1ENR_GPIOAEN_Pos			(0U)                              // RCC AHB1ENR register GPIOAN Bit Position
#define RCC_AHB1ENR_GPIOAEN_Msk 		(0x1 << RCC_AHB1ENR_GPIOAEN_Pos)  // RCC AHB1ENR register GPIOAN Bit Mask
#define RCC_AHB1ENR_GPIOAEN				RCC_AHB1ENR_GPIOAEN_Msk           // RCC AHB1ENR register GPIOAN Macro

#define RCC_AHB1ENR_GPIOBEN_Pos            (1U)                                // RCC AHB2ENR register GPIOBN Bit Position
#define RCC_AHB1ENR_GPIOBEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOBEN_Pos)  // RCC AHB2ENR register GPIOBN Bit Mask
#define RCC_AHB1ENR_GPIOBEN                RCC_AHB1ENR_GPIOBEN_Msk             // RCC AHB2ENR register GPIOBN Macro

#define RCC_AHB1ENR_GPIOCEN_Pos            (2U)
#define RCC_AHB1ENR_GPIOCEN_Msk            (0x1UL << RCC_AHB1ENR_GPIOCEN_Pos)
#define RCC_AHB1ENR_GPIOCEN                RCC_AHB1ENR_GPIOCEN_Msk

#define RCC_AHB1ENR_GPIODEN_Pos            (3U)
#define RCC_AHB1ENR_GPIODEN_Msk            (0x1UL << RCC_AHB1ENR_GPIODEN_Pos)
#define RCC_AHB1ENR_GPIODEN                RCC_AHB1ENR_GPIODEN_Msk

#define RCC_APB2ENR_SYSCFGEN_Pos		   (14U)
#define RCC_APB2ENR_SYSCFGEN_Msk		   (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)
#define RCC_APB2ENR_SYSCFGEN			   RCC_APB2ENR_SYSCFGEN_Msk

#define RCC_APB2ENR_SPI1EN_Pos			   (12U)
#define RCC_APB2ENR_SPI1EN_Msk			   (0x1UL << RCC_APB2ENR_SPI1EN_Pos)
#define RCC_APB2ENR_SPI1EN				   RCC_APB2ENR_SPI1EN_Msk

#define RCC_APB1ENR_SPI2EN_Pos             (14U)
#define RCC_APB1ENR_SPI2EN_Msk             (0x1UL << RCC_APB1ENR_SPI2EN_Pos)
#define RCC_APB1ENR_SPI2EN                 RCC_APB1ENR_SPI2EN_Msk

#define SPI_CR1_SPE						   (6U)


#include "RCC.h"
#include "GPIO.h"
#include "EXTI.h"
#include "SPI.h"

#endif /* INC_STM32F407XX_H_ */
