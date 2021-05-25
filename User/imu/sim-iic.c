#include "sim-iic.h"
#include "delay.h"


void IIC_Init(void) //初始化 IIC
{ 
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能 PB 端口时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //2M
    GPIO_Init(GPIOB, &GPIO_InitStructure); //根据设定参数初始化 GPIOB 
    IIC_SCL=1;
    IIC_SDA=1;
}

//产生 IIC 起始信号，在 SCL 高电平时，SDA 出现一个下调沿表示 I2C 启动信号
void IIC_Start(void)
{
    SDA_OUT(); //sda 线输出
    IIC_SDA=1; 
    IIC_SCL=1; //产生一个时钟
    delay_us(4);
    IIC_SDA=0; //当 SCL 在好电平期间 SDA 拉低
    delay_us(4);
    IIC_SCL=0; //钳住 I2C 总线，准备发送或接收数据
} 
//产生 IIC 停止信号
void IIC_Stop(void)
{
    SDA_OUT(); //sda 线输出
    IIC_SCL=0;
    IIC_SDA=0; 
    delay_us(4);
    IIC_SCL=1; //产生一个时钟
    delay_us(4);
    IIC_SDA=1; //发送 I2C 总线结束信号 
}

//返回值：1，接收应答失败
// 0，接收应答成功
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    SDA_IN(); //SDA 设置为输入 
    IIC_SDA=1;delay_us(1); 
    IIC_SCL=1;delay_us(1);
    while(READ_SDA)
    {
        ++ucErrTime;
        if(ucErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL=0; //时钟输出 0 
    return 0;
}

//产生 ACK 应答
void IIC_Ack(void)
{
    IIC_SCL=0; //时钟信号低电平
    SDA_OUT(); //输出
    IIC_SDA=0; //数据信号低电平
    delay_us(2);
    IIC_SCL=1; //时钟信号高电平
    delay_us(2);
    IIC_SCL=0; //时钟信号低电平
}
//不产生 ACK 应答 
void IIC_NAck(void)
{
    IIC_SCL=0; //时钟信号低电平
    SDA_OUT(); //输出
    IIC_SDA=1; //数据信号高电平
    delay_us(2);
    IIC_SCL=1; //时钟信号高电平
    delay_us(2);
    IIC_SCL=0; //时钟信号低电平
}

//返回从机有无应答
//1，有应答
//0，无应答 
void IIC_Send_Byte(u8 txd)
{ 
    u8 t; 
    SDA_OUT(); 
    IIC_SCL=0; //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    { 
        IIC_SDA=(txd&0x80)>>7;// 取字节的最高位的值，将其右移 7 位
        txd<<=1; //然后将 txd 数据左移一位，等待下一个循环时发送
        delay_us(2); //对 TEA5767 这三个延时都是必须的
        IIC_SCL=1; //产生一个时钟
        delay_us(2); 
        IIC_SCL=0;
        delay_us(2);
    }
}

//读 1 个字节，ack=1 时，发送 ACK，ack=0，发送 nACK 
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN(); //SDA 设置为输入
    for(i=0;i<8;i++ )
    {
        IIC_SCL=0; 
        delay_us(2);
        IIC_SCL=1; //产生一个时钟
        receive<<=1;//第一个循环左移不受影响，因为为 0，之后每循环一次左移一位
        if(READ_SDA) receive++; //READ_SDA 为读取输入脚的电平，为 1 的话将最低位置 1 
        delay_us(1); 
    }
    if (!ack) IIC_NAck(); //发送 nACK
    else IIC_Ack(); //发送 ACK 
    return receive;
}
