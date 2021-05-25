#include "iic.h"

static void I2C2_GPIO_Init()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void I2C_Config(I2C_TypeDef* I2Cx)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    I2C_InitTypeDef I2C_InitStruct;
    I2C_StructInit(&I2C_InitStruct);        //用结构体初始化函数初始化结构体
    I2C_InitStruct.I2C_ClockSpeed = 380000; //更改I2C速度为400kHz
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;//不允许允许应答
    I2C_Init(I2Cx, &I2C_InitStruct);
    I2C_Cmd(I2Cx, ENABLE);                  //使能I2C
}

void I2C2_Init()
{
    I2C2_GPIO_Init();
    I2C_Config(I2C2);
}

ErrorStatus I2C2_WaitForSuc(uint32_t I2C_EVENT, u16 waittime)
{
    ErrorStatus flag = ERROR;
    
    for(u16 i = 0; (flag == ERROR) && (i < waittime); ++i)
    {
        delay_us(1);
        flag = I2C_CheckEvent(I2C2, I2C_EVENT);
    }
    return flag;
}
