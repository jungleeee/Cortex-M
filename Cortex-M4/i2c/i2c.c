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
#include "stm32f4xx.h"
#include "stdio.h"
#include "i2c.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum i2cbus_i2cxEn {
    I2CBUS_I2CX_WRITE,
    I2CBUS_I2CX_READ
}i2cbus_i2cxTypeEnum;

typedef struct i2cbus_i2cxStr
{
    i2cbus_i2cxTypeEnum     i2cxType;                   /* i2c write or read */
    uint8_t                 slaveAddr;                  /* slave device address */
    uint8_t                 *dataBuf;                   /* write or read data buf */
    uint16_t                dataBufLen;                 /* write or read data length & buf length */

    void                    (*sendSuccessCback)(void);  /* write all data over callback */
    void                    (*recvSuccessCback)(void);  /* read all data over callback */
    void                    (*errDealCback)(void);      /* write or read data error callback */
}i2cbus_i2cxStruct;

/* Private define ------------------------------------------------------------*/
#define I2CBUS_I2CX_USE_DMA     0

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static i2cbus_i2cxStruct  i2cbus_i2c1Struct;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  i2cbus_i2c1Init
  * @param  None
  * @retval None
  */
void i2cbus_i2c1Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* GPIO set */
    RCC_AHB1PeriphClockCmd(I2CBUS_I2C1_SCL_PORT_Clk, ENABLE);
    RCC_AHB1PeriphClockCmd(I2CBUS_I2C1_SDA_PORT_Clk, ENABLE);

    GPIO_InitStructure.GPIO_Pin = I2CBUS_I2C1_SCL_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(I2CBUS_I2C1_SCL_Port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = I2CBUS_I2C1_SDA_Pin;
    GPIO_Init(I2CBUS_I2C1_SDA_Port, &GPIO_InitStructure);

    GPIO_PinAFConfig(I2CBUS_I2C1_SDA_Port, GPIO_PinSource7, GPIO_AF_I2C1);
    GPIO_PinAFConfig(I2CBUS_I2C1_SCL_Port, GPIO_PinSource6, GPIO_AF_I2C1);

    RCC_APB1PeriphClockCmd(I2CBUS_I2C1_Clk, ENABLE);

    I2C_DeInit(I2CBUS_I2C1);
    I2C_InitStruct.I2C_ClockSpeed = I2CBUS_I2C1_Speed;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = I2CBUS_I2C1_Addr;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2CBUS_I2C1, &I2C_InitStruct);

#if I2CBUS_I2CX_USE_DMA
	DMA_InitTypeDef DMA_InitStructure;

    /* DMA i2c rec */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                        /* Enable DMA CLK */

    DMA_DeInit(I2CBUS_I2C1_DMA_RX_Str);						                    /* DMA1 */
    while(DMA_GetCmdStatus(I2CBUS_I2C1_DMA_RX_Str) != DISABLE);

    DMA_InitStructure.DMA_Channel = I2CBUS_I2C1_DMA_RX_Chl;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&I2CBUS_I2C1->DR);	///* Peripheral address */
    DMA_InitStructure.DMA_Memory0BaseAddr = NULL;	                            ///* Memory address */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;				        ///* DMA dir Peripheral to Memory */
    DMA_InitStructure.DMA_dataBufferSize = 0;			                            ///* buffer size */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		    ///* No external increasing mode */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;			            ///* Allow memory increment mode */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	    ///* External data word length */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	    	    ///* Memory data word length */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;				                ///* Normal mode, non-cyclic mode */
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;			    	        ///* Set DMA priority */
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(I2CBUS_I2C1_DMA_RX_Str, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    /* DMA i2c send */
    DMA_DeInit(I2CBUS_I2C1_DMA_TX_Str);						                    /* DMA1 */
    while(DMA_GetCmdStatus(I2CBUS_I2C1_DMA_TX_Str) != DISABLE);

    DMA_InitStructure.DMA_Channel = I2CBUS_I2C1_DMA_TX_Chl;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				        ///* DMA dir Peripheral to Memory */
    DMA_Init(I2CBUS_I2C1_DMA_TX_Str, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    DMA_Cmd(I2CBUS_I2C1_DMA_RX_Str,ENABLE);
    DMA_ITConfig(I2CBUS_I2C1_DMA_RX_Str, DMA_IT_TC, ENABLE);
    DMA_ClearFlag(I2CBUS_I2C1_DMA_RX_Str, DMA_FLAG_TCIF5);

    DMA_Cmd(I2CBUS_I2C1_DMA_TX_Str,ENABLE);
    DMA_ITConfig(I2CBUS_I2C1_DMA_TX_Str, DMA_IT_TC, ENABLE);
    DMA_ClearFlag(I2CBUS_I2C1_DMA_TX_Str, DMA_FLAG_TCIF6);

    I2C_DMACmd(I2CBUS_I2C1, ENABLE);
#endif
    /* config I2C Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2CBUS_I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2CBUS_I2C1_EV_IRQ_Prio;/* pre-emption priority */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2CBUS_I2C1_EV_IRQ_SUB_Prio;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2CBUS_I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2CBUS_I2C1_ER_IRQ_Prio;/* pre-emption priority */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2CBUS_I2C1_ER_IRQ_SUB_Prio;
    NVIC_Init(&NVIC_InitStructure);

    I2C_Cmd(I2CBUS_I2C1, ENABLE);
	I2C_ITConfig(I2CBUS_I2C1, I2C_IT_BUF, ENABLE);                                 /* if ues I2C DMA, close I2C_IT_BUF */
	I2C_ITConfig(I2CBUS_I2C1, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
}

/**
  * @brief  i2cbus_dataBufByDMA
  * @param  i2cxType: write or read
  * @param  length:   write or read data length
  * @param  buf:      pointer of data buffer
  * @retval None
  */
void i2cbus_dataBufByDMA(const i2cbus_i2cxTypeEnum i2cxType, uint16_t length, uint8_t *buf)
{
    if(i2cxType == I2CBUS_I2CX_WRITE) {
        I2CBUS_I2C1_DMA_TX_Str->CR &= ((uint32_t)0xFFFFFFFE);    /* Disable DMA */
        I2CBUS_I2C1_DMA_TX_Str->M0AR = (uint32_t)buf;            /* set memory addr */
        I2CBUS_I2C1_DMA_TX_Str->NDTR = length;
        I2CBUS_I2C1_DMA_TX_Str->CR |= ((uint32_t)0x00000001);    /* Enable DMA */
    }
    else if(i2cxType == I2CBUS_I2CX_READ) {
        I2CBUS_I2C1_DMA_RX_Str->CR &= ((uint32_t)0xFFFFFFFE);    /* Disable DMA */
        I2CBUS_I2C1_DMA_RX_Str->M0AR = (uint32_t)buf;            /* set memory addr */
        I2CBUS_I2C1_DMA_RX_Str->NDTR = length;
        I2CBUS_I2C1_DMA_RX_Str->CR |= ((uint32_t)0x00000001);    /* Enable DMA */
    }
}

/**
  * @brief  write
  * @param  sAddr:  slave address
  * @param  buf:    data buffer
  * @param  length: write/read data length
  * @param  txSuccess: pointer of write complete callback
  * @param  errCback:  pointer of write/read error callback
  * @retval 0 start, 1 error
  */
int8_t i2cbus_i2c1StartWriteData(uint8_t sAddr,
                                   uint8_t *buf,
                                uint16_t length,
                        void (*txSuccess)(void),
                         void (*errCback)(void))
{
    i2cbus_i2c1Struct.i2cxType = I2CBUS_I2CX_WRITE;
    i2cbus_i2c1Struct.slaveAddr = sAddr;
    i2cbus_i2c1Struct.dataBuf = buf;
    i2cbus_i2c1Struct.dataBufLen = length;
    i2cbus_i2c1Struct.sendSuccessCback = txSuccess;
    i2cbus_i2c1Struct.recvSuccessCback = NULL;
    i2cbus_i2c1Struct.errDealCback = errCback;

    I2C_AcknowledgeConfig(I2CBUS_I2C1,DISABLE);
    /* Send I2C START condition */
    I2C_GenerateSTART(I2CBUS_I2C1, ENABLE);

    return 0;
}

/**
  * @brief  read
  * @param  sAddr:  slave address
  * @param  buf:    data buffer
  * @param  length: write/read data length
  * @param  rxSuccess: pointer of read complete callback
  * @param  errCback:  pointer of write/read error callback
  * @retval 0 start, 1 error
  */
int8_t i2cbus_i2c1StartReadData(uint8_t sAddr,
                                  uint8_t *buf,
                               uint16_t length,
                       void (*rxSuccess)(void),
                         void(*errCback)(void))
{
    i2cbus_i2c1Struct.i2cxType = I2CBUS_I2CX_READ;
    i2cbus_i2c1Struct.slaveAddr = sAddr;
    i2cbus_i2c1Struct.dataBuf = buf;
    i2cbus_i2c1Struct.dataBufLen = length;
    i2cbus_i2c1Struct.sendSuccessCback = NULL;
    i2cbus_i2c1Struct.recvSuccessCback = rxSuccess;
    i2cbus_i2c1Struct.errDealCback = errCback;

    I2C_AcknowledgeConfig(I2CBUS_I2C1,DISABLE);
    /* Send I2C START condition */
    I2C_GenerateSTART(I2CBUS_I2C1, ENABLE);

    return 0;
}

/**
  * @brief  Official documentation indicates that a higher priority is required
  * @param  PrePrio: 0
  * @param  SubPrio: 0
  * @retval None
  */
static void i2cbus_i2c1SetPrio(uint8_t PrePrio, uint8_t SubPrio)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = I2CBUS_I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PrePrio;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPrio;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  I2CBUS_I2C1_EVIRQ_Handler
  * @param  None
  * @retval None
  */
void I2CBUS_I2C1_EVIRQ_Handler(void)
{
    i2cbus_i2c1SetPrio(0, 0);
    uint32_t event = I2C_GetLastEvent(I2CBUS_I2C1);

    switch(event) {
        case I2C_EVENT_MASTER_MODE_SELECT: { /* send the START single, then enter this interrupt. EV5 */
            if(i2cbus_i2c1Struct.i2cxType == I2CBUS_I2CX_WRITE) {           /* start communication, write mode */
                I2C_Send7bitAddress(I2CBUS_I2C1, i2cbus_i2c1Struct.slaveAddr, I2C_Direction_Transmitter);
#if I2CBUS_I2CX_USE_DMA
                if(i2cbus_i2c1Struct.dataBufLen > 1) {
                    I2C_ITConfig(I2CBUS_I2C1, I2C_IT_BUF, DISABLE);         /* if ues I2C DMA, close I2C_IT_BUF */
                    i2cbus_dataBufByDMA(I2CBUS_I2CX_WRITE, i2cbus_i2c1Struct.dataBufLen, i2cbus_i2c1Struct.dataBuf)
                }
                else { /* if only send slave address/register, don't use DMA */
                    I2C_ITConfig(I2CBUS_I2C1, I2C_IT_BUF, ENABLE);
                }
#endif
            }
            else if(i2cbus_i2c1Struct.i2cxType == I2CBUS_I2CX_READ) {       /* start communication, read mode */
                I2C_Send7bitAddress(I2CBUS_I2C1, i2cbus_i2c1Struct.slaveAddr, I2C_Direction_Receiver);
#if I2CBUS_I2CX_USE_DMA
                if(i2cbus_i2c1Struct.dataBufLen > 1) {
                    I2C_DMALastTransferCmd(I2CBUS_I2C1, ENABLE);            /* Automatically sends NACK when the last one is received */
                    I2C_ITConfig(I2CBUS_I2C1, I2C_IT_BUF, DISABLE);
                    i2cbus_dataBufByDMA(I2CBUS_I2CX_READ, i2cbus_i2c1Struct.dataBufLen, i2cbus_i2c1Struct.dataBuf)
                }
                else { /* if only read one byte, don't use DMA */
                    I2C_ITConfig(I2CBUS_I2C1, I2C_IT_BUF, ENABLE);          /* if ues I2C DMA, close I2C_IT_BUF */
                }
#endif
            }
        }break;
        case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: { /* write mode, send slave address, then enter this interrupt. EV6 */
            if(i2cbus_i2c1Struct.dataBufLen == 0) {                         /* just send slave address */
                I2C_GenerateSTOP(I2CBUS_I2C1,ENABLE);

                if(i2cbus_i2c1Struct.sendSuccessCback != NULL) {            /* success callback */
                    i2cbus_i2c1SetPrio(I2CBUS_I2C1_EV_IRQ_Prio,I2CBUS_I2C1_EV_IRQ_SUB_Prio);
                    i2cbus_i2c1Struct.sendSuccessCback();
                }
            }
            else {
                I2CBUS_I2C1->DR = *(i2cbus_i2c1Struct.dataBuf);
                i2cbus_i2c1Struct.dataBuf++;
                i2cbus_i2c1Struct.dataBufLen--;
            }
        }break;
        case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED: {
            /* send mode, send slave address, then enter this interrupt. Prepare to receive data or configure ACK. EV6 */
            if(i2cbus_i2c1Struct.dataBufLen > 1) {                          /* set ACK */
                I2C_AcknowledgeConfig(I2CBUS_I2C1,ENABLE);
            }
            else if(i2cbus_i2c1Struct.dataBufLen == 1) {                    /* just receive one byte, set NACK */
                I2C_AcknowledgeConfig(I2CBUS_I2C1,DISABLE);
            }
        }break;
        case I2C_EVENT_MASTER_BYTE_RECEIVED: { /* receive one byte. EV7 */
            if(i2cbus_i2c1Struct.dataBufLen > 0) {                          /* recieve data */
                *(i2cbus_i2c1Struct.dataBuf) = (uint8_t)(I2CBUS_I2C1->DR);
                i2cbus_i2c1Struct.dataBuf++;
                i2cbus_i2c1Struct.dataBufLen--;
            }
            else {
                (void)(I2CBUS_I2C1->DR);
                return;
            }

            if(i2cbus_i2c1Struct.dataBufLen == 0) {                         /* all receive success */
                I2C_GenerateSTOP(I2CBUS_I2C1,ENABLE);

                if(i2cbus_i2c1Struct.recvSuccessCback != NULL) {            /* success callback */
                    i2cbus_i2c1SetPrio(I2CBUS_I2C1_EV_IRQ_Prio,I2CBUS_I2C1_EV_IRQ_SUB_Prio);
                    i2cbus_i2c1Struct.recvSuccessCback();
                }
            }
            else if(i2cbus_i2c1Struct.dataBufLen == 1) {
                I2C_AcknowledgeConfig(I2CBUS_I2C1, DISABLE);                /* last one byte, set NACK */
            }
        }break;
        case I2C_EVENT_MASTER_BYTE_TRANSMITTING: { /* Transmitting a Byte EV8 */

        }break;
        case I2C_EVENT_MASTER_BYTE_TRANSMITTED: { /* one byte send success, EV8_2 */
#if I2CBUS_I2CX_USE_DMA
            I2C_GenerateSTOP(I2CBUS_I2C1,ENABLE);
            if(i2cbus_i2c1Struct.sendSuccessCback != NULL) {                /* success callback */
                i2cbus_i2c1SetPrio(I2CBUS_I2C1_EV_IRQ_Prio,I2CBUS_I2C1_EV_IRQ_SUB_Prio);
                i2cbus_i2c1Struct.sendSuccessCback();
            }
#else
            if(i2cbus_i2c1Struct.dataBufLen == 0) {                         /* all send success */
                I2C_GenerateSTOP(I2CBUS_I2C1,ENABLE);

                if(i2cbus_i2c1Struct.sendSuccessCback != NULL) {            /* success callback */
                    i2cbus_i2c1SetPrio(I2CBUS_I2C1_EV_IRQ_Prio,I2CBUS_I2C1_EV_IRQ_SUB_Prio);
                    i2cbus_i2c1Struct.sendSuccessCback();
                }
            }
            else {
                I2CBUS_I2C1->DR = *(i2cbus_i2c1Struct.dataBuf);             /* send next one byte */
                i2cbus_i2c1Struct.dataBuf++;
                i2cbus_i2c1Struct.dataBufLen--;
            }
#endif
        }break;
    }
}
/**
  * @brief  I2CBUS_I2C1_ERIRQ_Handler
  * @param  None
  * @retval None
  */
void I2CBUS_I2C1_ERIRQ_Handler(void)
{
    if(I2C_GetITStatus(I2CBUS_I2C1,I2C_IT_BERR) != RESET) {  /* BERR */
        I2C_ClearITPendingBit(I2CBUS_I2C1,I2C_IT_BERR);
    }

    if(I2C_GetITStatus(I2CBUS_I2C1,I2C_IT_AF) != RESET) {    /* AF */
        I2C_ClearITPendingBit(I2CBUS_I2C1,I2C_IT_AF);
    }

    if(I2C_GetITStatus(I2CBUS_I2C1,I2C_IT_OVR) != RESET) {   /* OVR */
        I2C_ClearITPendingBit(I2CBUS_I2C1,I2C_IT_OVR);
    }

    if(I2C_GetITStatus(I2CBUS_I2C1,I2C_IT_ARLO) != RESET) {  /* ARLO */
        I2C_ClearITPendingBit(I2CBUS_I2C1,I2C_IT_ARLO);
    }
}

/**
  * @brief  I2C_DMA_TX_IRQHandler
  * @param  None
  * @retval None
  */
void I2C_DMA_TX_IRQHandler(void)
{
    if(DMA_GetITStatus(I2CBUS_I2C1_DMA_TX_Str,DMA_IT_TCIF6) != RESET) {
        DMA_ClearITPendingBit(I2CBUS_I2C1_DMA_TX_Str,DMA_IT_TCIF6);/* must clear IT flag */
    }
}

/**
  * @brief  I2C_DMA_RX_IRQHandler
  * @param  None
  * @retval None
  */
void I2C_DMA_RX_IRQHandler(void)
{
    if(DMA_GetITStatus(I2CBUS_I2C1_DMA_RX_Str,DMA_IT_TCIF5) != RESET) {
        DMA_ClearITPendingBit(I2CBUS_I2C1_DMA_RX_Str,DMA_IT_TCIF5);/* must clear IT flag */

        I2C_GenerateSTOP(I2CBUS_I2C1,ENABLE);
    }
}

/**
  * @}
  */

/************************ Copyright (C) jungleeee 2017 *****END OF FILE****/
