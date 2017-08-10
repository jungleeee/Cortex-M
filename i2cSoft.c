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
#define i2c1_SCL_out_High   GPIO_SetBits(I2C1_gpio_port, I2C1_SCL_pin)
#define i2c1_SCL_out_Low    GPIO_ResetBits(I2C1_gpio_port, I2C1_SCL_pin)
#define i2c1_SDA_out_High   GPIO_SetBits(I2C1_gpio_port, I2C1_SDA_pin)
#define i2c1_SDA_out_Low    GPIO_ResetBits(I2C1_gpio_port, I2C1_SDA_pin)
#define i2c1_SDA_in_Bit     GPIO_ReadInputDataBit(I2C1_gpio_port, I2C1_SDA_pin)

#define i2c1_SDA_set_out_mode       {I2C1_gpio_port->MODER &= ~(GPIO_MODER_MODER0 << (7 * 2));\
                                    I2C1_gpio_port->MODER |= (((uint32_t)GPIO_Mode_OUT) << (7 * 2));}
#define i2c1_SDA_set_in_mode        {I2C1_gpio_port->MODER &= ~(GPIO_MODER_MODER0 << (7 * 2));\
                                    I2C1_gpio_port->MODER |= (((uint32_t)GPIO_Mode_IN) << (7 * 2));}

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
  * @brief  IIC_Init
  * @param  None
  * @retval None
  */
void IIC_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = I2C1_SCL_pin | I2C1_SDA_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(I2C1_gpio_port, &GPIO_InitStructure);

    i2c1_SCL_out_High;
    i2c1_SDA_out_High;
}

/**
  * @brief  IIC_Start
  * @param  None
  * @retval None
  */
void IIC_Start(void)
{
	i2c1_SDA_set_out_mode;
    i2c1_SCL_out_High;
    i2c1_SDA_out_High;
	delay_us(4);
 	i2c1_SDA_out_Low;
	delay_us(4);
	i2c1_SCL_out_Low;
}

/**
  * @brief  IIC_Stop
  * @param  None
  * @retval None
  */
void IIC_Stop(void)
{
	i2c1_SDA_set_out_mode;
	i2c1_SCL_out_Low;
	i2c1_SDA_out_Low;
 	delay_us(4);
    i2c1_SCL_out_High;
    i2c1_SDA_out_High;
	delay_us(4);
}

/**
  * @brief  IIC_Wait_Ack
  * @param  None
  * @retval 1 failed, 0 success
  */
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrCount = 0;

	i2c1_SDA_set_in_mode;
	i2c1_SDA_out_High;
    delay_us(1);
	i2c1_SCL_out_High;
    delay_us(1);
	while(i2c1_SDA_in_Bit) {
		ucErrCount++;
		if(ucErrCount > 250) {
			IIC_Stop();

			return 1;
		}
	}

	i2c1_SCL_out_Low;

    return 0;
}

/**
  * @brief  IIC_Ack
  * @param  None
  * @retval None
  */
void IIC_Ack(void)
{
	i2c1_SCL_out_Low;
	i2c1_SDA_set_out_mode;
	i2c1_SDA_out_Low;
	delay_us(2);
	i2c1_SCL_out_High;
	delay_us(2);
	i2c1_SCL_out_Low;
}

/**
  * @brief  IIC_NAck
  * @param  None
  * @retval None
  */
void IIC_NAck(void)
{
	i2c1_SCL_out_Low;
	i2c1_SDA_set_out_mode;
	i2c1_SDA_out_High;
	delay_us(2);
	i2c1_SCL_out_High;
	delay_us(2);
	i2c1_SCL_out_Low;
}

/**
  * @brief  IIC_Send_Byte
  * @param  None
  * @retval None
  */
void IIC_Send_Byte(uint8_t byte)
{
	i2c1_SDA_set_out_mode;
    i2c1_SCL_out_Low;

    for(int i = 0;i < 8;i++) {
        (byte & (0x80 >> i)) ? i2c1_SDA_out_High : i2c1_SDA_out_Low;
		delay_us(2);
		i2c1_SCL_out_High;
		delay_us(2);
		i2c1_SCL_out_Low;
		delay_us(2);
    }
}

/**
  * @brief  IIC_Read_Byte
  * @param  ack: 1 send ack; 0 send Nack
  * @retval None
  */
uint8_t IIC_Read_Byte(unsigned char ack)
{
	uint8_t byte = 0;

	i2c1_SDA_set_in_mode;

    for(int i = 0;i < 8;i++) {
        i2c1_SCL_out_Low;//IIC_SCL = 0;
        delay_us(2);
		i2c1_SCL_out_High;//IIC_SCL = 1;
        byte <<= 1;
        if(i2c1_SDA_in_Bit)
            byte++;
		delay_us(1);
    }

    if(!ack)
        IIC_NAck();
    else
        IIC_Ack();

    return byte;
}

/**
  * @brief  IIC_Write_Len
  * @param  addr : slave addr
  * @param  reg  : reg addr
  * @param  len  : data length
  * @param  buf  : pointer of data buf
  * @retval 0 ok
  */
uint8_t IIC_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);
    if(IIC_Wait_Ack()) {
		IIC_Stop();

		return 1;
	}
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
	for(int i = 0;i < len;i++) {
		IIC_Send_Byte(buf[i]);
        if(IIC_Wait_Ack()) {
			IIC_Stop();

			return 1;
		}
	}

    IIC_Stop();

	return 0;
}

/**
  * @brief  IIC_Read_Len
  * @param  addr : slave addr
  * @param  reg  : reg addr
  * @param  len  : data length
  * @param  buf  : pointer of data buf
  * @retval 0 ok
  */
uint8_t IIC_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
 	IIC_Start();
	IIC_Send_Byte((addr << 1) | 0);
	if(IIC_Wait_Ack()) {
		IIC_Stop();

		return 1;
	}
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
	IIC_Send_Byte((addr << 1) | 1);
    IIC_Wait_Ack();
	while(len) {
		if(len == 1)
            *buf = IIC_Read_Byte(0);
		else
            *buf = IIC_Read_Byte(1);

		len--;
		buf++;
	}

    IIC_Stop();

	return 0;
}

/**
  * @}
  */

/************************ Copyright (C) jungleeee 2017 *****END OF FILE****/
