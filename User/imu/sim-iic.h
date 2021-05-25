#ifndef __MYIIC_H
#define __MYIIC_H

#ifdef  __cplusplus
    extern "C" {
#endif  //__cplusplus

#include "sys.h"
 
// IO 方向设置
#define SDA_IN() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}
// IO 操作函数
#define IIC_SCL PBout(10) //SCL
#define IIC_SDA PBout(11) //SDA
#define READ_SDA PBin(11) //输入 SDA 
// IIC 所有操作函数
void IIC_Init(void); //初始化 IIC 的 IO 口
void IIC_Start(void); //发送 IIC 开始信号
void IIC_Stop(void); //发送 IIC 停止信号
void IIC_Send_Byte(u8 txd); //IIC 发送一个字节
u8 IIC_Read_Byte(unsigned char ack); //IIC 读取一个字节
u8 IIC_Wait_Ack(void); //IIC 等待 ACK 信号
void IIC_Ack(void); //IIC 发送 ACK 信号
void IIC_NAck(void); //IIC 不发送 ACK 信号
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);//写一个字节
u8 IIC_Read_One_Byte(u8 daddr,u8 addr); //读一个字节

#ifdef  __cplusplus
}
#endif //__cplusplus

#endif

