/**
  ******************************************************************************
  * @file    tim.c
  * @author  Jungle
  * @version V1.0
  * @date    2017/9/15
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <tim.h>

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  tim_ws2812GRBInit
  * @param  None
  * @retval None
  */
void tim_ws2812GRBInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructre;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(tim_ws2812GRB_gpio_port_clk, ENABLE);
    RCC_APB1PeriphClockCmd(tim_ws2812GRB_tim_clk, ENABLE);                     /* TIM5 APB1 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                       /* Enable DMA CLK */

    GPIO_InitStructre.GPIO_Pin = tim_ws2812GRB_gpio_pin;
    GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructre.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(tim_ws2812GRB_gpio_port,&GPIO_InitStructre);

    GPIO_PinAFConfig(tim_ws2812GRB_gpio_port,GPIO_PinSource1,GPIO_AF_TIM5);

    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;                                /* 1/84 = 0.0119047619047619us */
    TIM_TimeBaseInitStructure.TIM_Period = 105;                                 /* 0.0119047619047619 * 105 = 1.254us //71 34 */
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;jjj
    TIM_TimeBaseInit(tim_ws2812GRB_tim,&TIM_TimeBaseInitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC2Init(tim_ws2812GRB_tim,&TIM_OCInitStructure);

    TIM_SelectCCDMA(tim_ws2812GRB_tim, ENABLE);
    TIM_DMACmd(tim_ws2812GRB_tim, TIM_DMA_Update, ENABLE);
    TIM_UpdateRequestConfig(tim_ws2812GRB_tim, TIM_UpdateSource_Regular);
    TIM_UpdateDisableConfig(tim_ws2812GRB_tim, DISABLE);

    /* CH6£¬DMA TIM */
    DMA_DeInit(tim_ws2812GRB_tim_DMA_Str);						                /* DMA1 */
    while(DMA_GetCmdStatus(tim_ws2812GRB_tim_DMA_Str) != DISABLE);

    DMA_InitStructure.DMA_Channel = tim_ws2812GRB_tim_DMA_Chl;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM5->CCR2;	        /* Peripheral address */
    DMA_InitStructure.DMA_Memory0BaseAddr = NULL;                               /* Memory address */
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				        /* DMA dir Memory To Peripheral */
    DMA_InitStructure.DMA_BufferSize = 1;	                                    /* length */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		    /* Peripheral address not add */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;			            /* Memory address add */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;     /* Peripheral data length */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;             /* Memory data length */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;				                /* normal mode */
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                       /* DMA Priority */
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(tim_ws2812GRB_tim_DMA_Str, &DMA_InitStructure);

    //NVIC_InitStructure.NVIC_IRQChannel = tim_ws2812GRB_tim_DMA_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);

    DMA_Cmd(tim_ws2812GRB_tim_DMA_Str, ENABLE);
    DMA_ITConfig(tim_ws2812GRB_tim_DMA_Str, DMA_IT_TC, DISABLE);
    DMA_ClearITPendingBit(tim_ws2812GRB_tim_DMA_Str, DMA_IT_TCIF6);             /* must clear */

    TIM_Cmd(tim_ws2812GRB_tim,ENABLE);                                          /* Enable */
    TIM_ITConfig(tim_ws2812GRB_tim, TIM_IT_Update, DISABLE);                    /* Close  Updata Interrupt */
    TIM_ClearFlag(tim_ws2812GRB_tim, TIM_FLAG_Update);                          /* clear  Updata Interrupt flag */
}

/**
  * @brief  tim_CCRxSetByDMA
  * @param  TIMx: tim1-tim8
  * @param  length: data length
  * @param  dataBuf: pointer of data
  * @retval None
  */
void tim_CCRxSetByDMA(const TIM_TypeDef* TIMx, uint16_t length, uint32_t *dataBuf)
{
    if(TIMx == tim_ws2812GRB_tim) {
        DMA_ClearITPendingBit(tim_ws2812GRB_tim_DMA_Str,DMA_IT_TCIF6);          /* must clear IT flag */
        tim_ws2812GRB_tim_DMA_Str->CR &= ((uint32_t)0xFFFFFFFE);                /* Disable DMA */
        tim_ws2812GRB_tim_DMA_Str->M0AR = (uint32_t)dataBuf;                    /* set memory addr */
        tim_ws2812GRB_tim_DMA_Str->NDTR = length;
        tim_ws2812GRB_tim_DMA_Str->CR |= ((uint32_t)0x00000001);                /* Enable DMA */
    }
}

/**
  * @brief  tim_ws2812GRB_tim_DMA_IRQHandler
  * @param  None
  * @retval None
  */
void tim_ws2812GRB_tim_DMA_IRQHandler(void)
{
    if(DMA_GetITStatus(tim_ws2812GRB_tim_DMA_Str,DMA_IT_TCIF6) != RESET) {
        DMA_ClearITPendingBit(tim_ws2812GRB_tim_DMA_Str, DMA_IT_TCIF6);         /* must clear IT flag */
    }
}

/**
  * @}
  */

/************************ Copyright (C) Jungleeee 2017 *****END OF FILE****/

