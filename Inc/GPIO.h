

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f407xx.h"

/*
 * @def_Group GPIO_pins_define
 *
 */

#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */

/*
 * @def_Group GPIO_mode_define
 *
 */
#define GPIO_MODE_INPUT 		(0x0U)
#define GPIO_MODE_OUTPUT		(0x1U)
#define GPIO_MODE_AF			(0x2U)
#define GPIO_MODE_ANALOG		(0x3U)

/*
 *  @def_Group GPIO_OTYPE_Modes
 *
 */

#define GPIO_OTYPE_PP		(0x0U)
#define GPIO_OTYPE_OD		(0x1U)

/*
 * @def_Group GPIO_pull_define
 */

#define GPIO_PUPD_NOPULL		(0x0U)
#define GPIO_PUPD_PULLUP		(0x1U)
#define GPIO_PUPD_PULLDOWN		(0x2U)

/*
 * @def_Group GPIO_speed_define
 */

#define GPIO_SPEED_LOW			(0x0U)
#define GPIO_SPEED_MEDIUM		(0x1U)
#define GPIO_SPEED_HIGH			(0x2U)
#define GPIO_SPEED_VERY			(0x3U)


typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState_t;

typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Otype;     /*!< GPIO Pin Numbers @def_Group GPIO_OTYPE_Modes */


  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins.
                            This parameter can be a value of @ref GPIO_Alternate_function_selection */
}GPIO_InitTypeDef_t;


void  GPIO_Init(GPIO_TypeDef_t  *GPIOx, GPIO_InitTypeDef_t *GPIO_Init);
void GPIO_WritePin(GPIO_TypeDef_t* GPIOx, uint16_t GPIO_Pin, GPIO_PinState_t PinState);
GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t* GPIOx, uint16_t GPIO_Pin);
void GPIO_LockPin(GPIO_TypeDef_t* GPIOx, uint16_t GPIO_Pin);
void GPIO_TogglePin(GPIO_TypeDef_t* GPIOx, uint16_t GPIO_Pin);


#endif /* INC_GPIO_H_ */
