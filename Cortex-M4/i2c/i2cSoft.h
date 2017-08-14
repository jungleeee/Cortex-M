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
#define i2cBus_SCL_Pin            GPIO_Pin_6
#define i2cBus_SDA_Pin            GPIO_Pin_7
#define i2cBus_GPIO_Port          GPIOB
#define i2cBus_GPIO_PORT_Clk      RCC_AHB1Periph_GPIOB
#define i2cBus_Clk                RCC_APB1Periph_I2C1

/* Exported functions ------------------------------------------------------- */
void i2cBus_init(void);
void i2cBus_sendStart(void);
void i2cBus_sendStop(void);
void i2cBus_sendOneByte(uint8_t byte);
void i2cBus_sendAck(void);
void i2cBus_sendNack(void);
uint8_t i2cBus_readOneByte(unsigned char ack);
uint8_t i2cBus_waitAck(void);

/**
  * @}
  */

#endif /* __I2C_H */

/************************ Copyright (C) jungleeee 2017 *****END OF FILE****/
