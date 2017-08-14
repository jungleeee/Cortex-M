/**
  ******************************************************************************
  * @file    i2c.c
  * @author  Jungle
  * @version V1.0
  * @date    2017/8/9
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx.h>
#include <i2c.h>

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define i2cBus_SCL_OUT_High   GPIO_SetBits(i2cBus_GPIO_Port, i2cBus_SCL_Pin)
#define i2cBus_SCL_OUT_Low    GPIO_ResetBits(i2cBus_GPIO_Port, i2cBus_SCL_Pin)
#define i2cBus_SDA_OUT_High   GPIO_SetBits(i2cBus_GPIO_Port, i2cBus_SDA_Pin)
#define i2cBus_SDA_OUT_Low    GPIO_ResetBits(i2cBus_GPIO_Port, i2cBus_SDA_Pin)
#define i2cBus_SDA_IN_Bit     GPIO_ReadInputDataBit(i2cBus_GPIO_Port, i2cBus_SDA_Pin)

#define i2cBus_SDA_SET_OUT_Mode       {i2cBus_GPIO_Port->MODER &= ~(GPIO_MODER_MODER0 << (7 * 2));\
                                    i2cBus_GPIO_Port->MODER |= (((uint32_t)GPIO_Mode_OUT) << (7 * 2));}
#define i2cBus_SDA_SET_IN_Mode        {i2cBus_GPIO_Port->MODER &= ~(GPIO_MODER_MODER0 << (7 * 2));\
                                    i2cBus_GPIO_Port->MODER |= (((uint32_t)GPIO_Mode_IN) << (7 * 2));}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  delay_us
  * @param  None
  * @retval None
  */
void delay_us(uint8_t t)
{
    for(int i = 0;i < t;i++) {
        for(int j = 0;j < 8;j++) {
        }
    }
}

/**
  * @brief  i2cBus_init
  * @param  None
  * @retval None
  */
void i2cBus_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = i2cBus_SCL_Pin | i2cBus_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(i2cBus_GPIO_Port, &GPIO_InitStructure);

    i2cBus_SCL_OUT_High;
    i2cBus_SDA_OUT_High;
}

/**
  * @brief  i2cBus_sendStart
  * @param  None
  * @retval None
  */
void i2cBus_sendStart(void)
{
	i2cBus_SDA_SET_OUT_Mode;
    i2cBus_SCL_OUT_High;
    i2cBus_SDA_OUT_High;
	delay_us(4);
 	i2cBus_SDA_OUT_Low;
	delay_us(4);
	i2cBus_SCL_OUT_Low;
}

/**
  * @brief  i2cBus_sendStop
  * @param  None
  * @retval None
  */
void i2cBus_sendStop(void)
{
	i2cBus_SDA_SET_OUT_Mode;
	i2cBus_SCL_OUT_Low;
	i2cBus_SDA_OUT_Low;
 	delay_us(4);
    i2cBus_SCL_OUT_High;
    i2cBus_SDA_OUT_High;
	delay_us(4);
}

/**
  * @brief  i2cBus_waitAck
  * @param  None
  * @retval 1 failed, 0 success
  */
uint8_t i2cBus_waitAck(void)
{
	uint8_t ucErrCount = 0;

	i2cBus_SDA_SET_IN_Mode;
	i2cBus_SDA_OUT_High;
    delay_us(1);
	i2cBus_SCL_OUT_High;
    delay_us(1);
	while(i2cBus_SDA_IN_Bit) {
		ucErrCount++;
		if(ucErrCount > 250) {
			i2cBus_sendStop();

			return 1;
		}
	}

	i2cBus_SCL_OUT_Low;

    return 0;
}

/**
  * @brief  i2cBus_sendAck
  * @param  None
  * @retval None
  */
void i2cBus_sendAck(void)
{
	i2cBus_SCL_OUT_Low;
	i2cBus_SDA_SET_OUT_Mode;
	i2cBus_SDA_OUT_Low;
	delay_us(2);
	i2cBus_SCL_OUT_High;
	delay_us(2);
	i2cBus_SCL_OUT_Low;
}

/**
  * @brief  i2cBus_sendNack
  * @param  None
  * @retval None
  */
void i2cBus_sendNack(void)
{
	i2cBus_SCL_OUT_Low;
	i2cBus_SDA_SET_OUT_Mode;
	i2cBus_SDA_OUT_High;
	delay_us(2);
	i2cBus_SCL_OUT_High;
	delay_us(2);
	i2cBus_SCL_OUT_Low;
}

/**
  * @brief  i2cBus_sendOneByte
  * @param  None
  * @retval None
  */
void i2cBus_sendOneByte(uint8_t byte)
{
	i2cBus_SDA_SET_OUT_Mode;
    i2cBus_SCL_OUT_Low;

    for(int i = 0;i < 8;i++) {
        (byte & (0x80 >> i)) ? i2cBus_SDA_OUT_High : i2cBus_SDA_OUT_Low;
		delay_us(2);
		i2cBus_SCL_OUT_High;
		delay_us(2);
		i2cBus_SCL_OUT_Low;
		delay_us(2);
    }
}

/**
  * @brief  i2cBus_readOneByte
  * @param  ack: 1 send ack; 0 send Nack
  * @retval None
  */
uint8_t i2cBus_readOneByte(unsigned char ack)
{
	uint8_t byte = 0;

	i2cBus_SDA_SET_IN_Mode;

    for(int i = 0;i < 8;i++) {
        i2cBus_SCL_OUT_Low;
        delay_us(2);
        i2cBus_SCL_OUT_High;
        byte <<= 1;
        if(i2cBus_SDA_IN_Bit)
            byte++;
		delay_us(1);
    }

    if(!ack)
        i2cBus_sendNack();
    else
        i2cBus_sendAck();

    return byte;
}

/**
  * @brief  i2cBus_writeLength
  * @param  addr : slave addr
  * @param  reg  : reg addr
  * @param  len  : data length
  * @param  buf  : pointer of data buf
  * @retval 0 ok
  */
uint8_t i2cBus_writeLength(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    i2cBus_sendStart();
	i2cBus_sendOneByte((addr << 1) | 0);
    if(i2cBus_waitAck()) {
        i2cBus_sendStop();

        return 1;
	}
    i2cBus_sendOneByte(reg);
    i2cBus_waitAck();
	for(int i = 0;i < len;i++) {
		i2cBus_sendOneByte(buf[i]);
        if(i2cBus_waitAck()) {
			i2cBus_sendStop();

			return 1;
		}
	}

    i2cBus_sendStop();

	return 0;
}

/**
  * @brief  i2cBus_readLength
  * @param  addr : slave addr
  * @param  reg  : reg addr
  * @param  len  : data length
  * @param  buf  : pointer of data buf
  * @retval 0 ok
  */
uint8_t i2cBus_readLength(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
 	i2cBus_sendStart();
	i2cBus_sendOneByte((addr << 1) | 0);
	if(i2cBus_waitAck()) {
		i2cBus_sendStop();

		return 1;
	}
    i2cBus_sendOneByte(reg);
    i2cBus_waitAck();
    i2cBus_sendStart();
	i2cBus_sendOneByte((addr << 1) | 1);
    i2cBus_waitAck();
	while(len) {
		if(len == 1)
            *buf = i2cBus_readOneByte(0);
		else
            *buf = i2cBus_readOneByte(1);

		len--;
		buf++;
	}

    i2cBus_sendStop();

	return 0;
}

/**
  * @}
  */

/************************ Copyright (C) jungleeee 2017 *****END OF FILE****/
