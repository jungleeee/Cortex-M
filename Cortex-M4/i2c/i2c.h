/**
  ******************************************************************************
  * @file    i2c.h
  * @author  Jungle
  * @version V1.0
  * @date    2017/8/9
  * @brief
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H
#define __I2C_H

/* Includes FreeRTOS----------------------------------------------------------*/

/* Includes STM32 3.5---------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define I2CBUS_I2C1                         I2C1
#define I2CBUS_I2C1_SCL_Pin                 GPIO_Pin_6
#define I2CBUS_I2C1_SCL_Port                GPIOB
#define I2CBUS_I2C1_SCL_PORT_Clk            RCC_AHB1Periph_GPIOB
#define I2CBUS_I2C1_SDA_Pin                 GPIO_Pin_7
#define I2CBUS_I2C1_SDA_Port                GPIOB
#define I2CBUS_I2C1_SDA_PORT_Clk            RCC_AHB1Periph_GPIOB
#define I2CBUS_I2C1_Clk                     RCC_APB1Periph_I2C1

#define I2CBUS_I2C1_EV_IRQn                 I2C1_EV_IRQn
#define I2CBUS_I2C1_EV_IRQ_Prio             8
#define I2CBUS_I2C1_EV_IRQ_SUB_Prio         0
#define I2CBUS_I2C1_EVIRQ_Handler           I2C1_EV_IRQHandler

#define I2CBUS_I2C1_ER_IRQn                 I2C1_ER_IRQn
#define I2CBUS_I2C1_ER_IRQ_Prio             8
#define I2CBUS_I2C1_ER_IRQ_SUB_Prio         0
#define I2CBUS_I2C1_ERIRQ_Handler           I2C1_ER_IRQHandler

#define I2CBUS_I2C1_Speed                   200000
#define I2CBUS_I2C1_Addr                    0x25

#define I2CBUS_I2C1_DMA_RX_Str              DMA1_Stream5
#define I2CBUS_I2C1_DMA_RX_Chl              DMA_Channel_1
#define I2CBUS_I2C1_DMA_RX_IRQn             DMA1_Stream5_IRQn
#define I2CBUS_I2C1_DMA_RX_IRQHandler       DMA1_Stream5_IRQHandler

#define I2CBUS_I2C1_DMA_TX_Str              DMA1_Stream6
#define I2CBUS_I2C1_DMA_TX_Chl              DMA_Channel_1
#define I2CBUS_I2C1_DMA_TX_IRQn             DMA1_Stream6_IRQn
#define I2CBUS_I2C1_DMA_TX_IRQHandler       DMA1_Stream6_IRQHandler

/* Exported functions ------------------------------------------------------- */
void   i2cbus_i2c1Init(void);
int8_t i2cbus_i2c1StartWriteData(uint8_t sAddr,
                                   uint8_t *buf,
                                uint16_t length,
                        void (*txSuccess)(void),
                         void (*errCback)(void));
int8_t i2cbus_i2c1StartReadData(uint8_t sAddr,
                                  uint8_t *buf,
                               uint16_t length,
                       void (*rxSuccess)(void),
                         void(*errCback)(void));

/**
  * @}
  */

#endif /* __I2C_H */

/************************ Copyright (C) jungleeee 2017 *****END OF FILE****/
