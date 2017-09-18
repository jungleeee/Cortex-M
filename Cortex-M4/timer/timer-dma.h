/**
  ******************************************************************************
  * @file    tim.h
  * @author  Jungle
  * @version V1.0
  * @date    2017/9/15
  * @brief
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H
#define __TIM_H

/* Includes STM32 3.5---------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define tim_ws2812GRB_gpio_pin              GPIO_Pin_1
#define tim_ws2812GRB_gpio_port             GPIOA
#define tim_ws2812GRB_gpio_port_clk         RCC_AHB1Periph_GPIOA
#define tim_ws2812GRB_tim                   TIM5
#define tim_ws2812GRB_tim_clk               RCC_APB1Periph_TIM5
#define tim_ws2812GRB_tim_DMA_Str           DMA1_Stream6
#define tim_ws2812GRB_tim_DMA_Chl           DMA_Channel_6
#define tim_ws2812GRB_tim_DMA_IRQn          DMA1_Stream6_IRQn
#define tim_ws2812GRB_tim_DMA_IRQHandler    DMA1_Stream6_IRQHandler

/* Exported functions ------------------------------------------------------- */
void tim_ws2812GRBInit(void);
void tim_CCRxSetByDMA(const TIM_TypeDef* TIMx, uint16_t length, uint32_t *dataBuf);

#endif /* __TIM_H */

/************************ Copyright (C) Jungleeee 2017 *****END OF FILE****/

