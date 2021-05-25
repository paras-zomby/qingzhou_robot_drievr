#ifndef __IIC_H__
#define __IIC_H__

#ifdef  __cplusplus
    extern "C" {
#endif  //__cplusplus

#include "stm32f10x.h"
#include "delay.h"

#define I2C2_START I2C_GenerateSTART(I2C2, ENABLE);
#define I2C2_SENDADDR_W(_addr) I2C_Send7bitAddress(I2C2, _addr, I2C_Direction_Transmitter);
#define I2C2_SENDADDR_R(_addr) I2C_Send7bitAddress(I2C2, _addr, I2C_Direction_Receiver);
#define I2C2_RECEIVEDATA I2C_ReceiveData(I2C2);
#define I2C2_SENDDATA(data_ptr) I2C_SendData(I2C2, data_ptr);
#define I2C2_STOP I2C_GenerateSTOP(I2C2, ENABLE);


void I2C2_Init(void);
ErrorStatus I2C2_WaitForSuc(uint32_t I2C_EVENT, u16 waittime_us);


#ifdef  __cplusplus
}
#endif //__cplusplus

#endif  //__IIC_H__
