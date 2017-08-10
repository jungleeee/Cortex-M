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
#define I2C1_SCL_pin            GPIO_Pin_6
#define I2C1_SDA_pin            GPIO_Pin_7
#define I2C1_gpio_port          GPIOB
#define I2C1_gpio_port_clk      RCC_AHB1Periph_GPIOB
#define I2C1_clk                RCC_APB1Periph_I2C1
#define I2C1_speed              400000
#define I2C1OwnAddr             0x00

/* Exported functions ------------------------------------------------------- */
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t byte);
void IIC_Ack(void);
void IIC_NAck(void);
uint8_t IIC_Read_Byte(unsigned char ack);
uint8_t IIC_Wait_Ack(void);

/**
  * @}
  */

#endif /* __I2C_H */

/************************ Copyright (C) jungleeee 2017 *****END OF FILE****/
