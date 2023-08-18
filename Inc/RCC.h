

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "stm32f407xx.h"

#define RCC_GPIOA_CLK_ENABLE()			do{uint32_t tempValue = 0;\
	                                       SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);\
	                                       tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);\
	                                       UNUSED(tempValue);\
										   }while(0)

#define RCC_GPIOB_CLK_ENABLE()       do { \
                                        __IO uint32_t tempValue = 0x00U; \
                                        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
                                        tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);\
                                        UNUSED(tempValue); \
                                          } while(0U)
#define RCC_GPIOC_CLK_ENABLE()     do { \
                                        __IO uint32_t tempValue = 0x00U; \
                                        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
										tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);\
                                        UNUSED(tempValue); \
                                          } while(0U)
#define RCC_GPIOD_CLK_ENABLE()       do { \
                                        __IO uint32_t tempValue = 0x00U; \
                                        SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
										tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);\
                                        UNUSED(tempValue); \
                                         } while(0U)

#define RCC_SYSCFG_CLK_ENABLE()   do { \
                                        __IO uint32_t tempValue = 0x00U; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
										tempValue = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);\
                                        UNUSED(tempValue); \
                                    } while(0U)

/*
 * APB1
 * */

#define RCC_SPI1_CLK_ENABLE()     do { \
                                        __IO uint32_t tempValue = 0x00U; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
										tempValue = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);\
                                        UNUSED(tempValue); \
                                    } while(0U)

#define RCC_SPI1_CLK_DISABLE()        CLEAR_BIT(RCC->APB2ENR,RCC_APB2ENR_SPI1EN);

/*
 * APB2
 * */

#define RCC_SPI2_CLK_ENABLE()      do { \
                                        __IO uint32_t tempValue = 0x00U; \
                                        SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);\
                                        /* Delay after an RCC peripheral clock enabling */ \
		                                tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);\
                                        UNUSED(tempValue); \
                                          } while(0U)

#define RCC_SPI2_CLK_DISABLE()		  CLEAR_BIT(RCC->APB1ENR,RCC_APB1ENR_SPI2EN);



#define RCC_GPIOA_CLK_DISABLE()        CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOAEN);
#define RCC_GPIOB_CLK_DISABLE()        CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOBEN);
#define RCC_GPIOC_CLK_DISABLE()        CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIOCEN);
#define RCC_GPIOD_CLK_DISABLE()		   CLEAR_BIT(RCC->AHB1ENR,RCC_AHB1ENR_GPIODEN);

#endif /* INC_RCC_H_ */
