#include "sim-iic.h"
#include "delay.h"


void IIC_Init(void) //��ʼ�� IIC
{ 
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ�� PB �˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //2M
    GPIO_Init(GPIOB, &GPIO_InitStructure); //�����趨������ʼ�� GPIOB 
    IIC_SCL=1;
    IIC_SDA=1;
}

//���� IIC ��ʼ�źţ��� SCL �ߵ�ƽʱ��SDA ����һ���µ��ر�ʾ I2C �����ź�
void IIC_Start(void)
{
    SDA_OUT(); //sda �����
    IIC_SDA=1; 
    IIC_SCL=1; //����һ��ʱ��
    delay_us(4);
    IIC_SDA=0; //�� SCL �ںõ�ƽ�ڼ� SDA ����
    delay_us(4);
    IIC_SCL=0; //ǯס I2C ���ߣ�׼�����ͻ��������
} 
//���� IIC ֹͣ�ź�
void IIC_Stop(void)
{
    SDA_OUT(); //sda �����
    IIC_SCL=0;
    IIC_SDA=0; 
    delay_us(4);
    IIC_SCL=1; //����һ��ʱ��
    delay_us(4);
    IIC_SDA=1; //���� I2C ���߽����ź� 
}

//����ֵ��1������Ӧ��ʧ��
// 0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    SDA_IN(); //SDA ����Ϊ���� 
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
    IIC_SCL=0; //ʱ����� 0 
    return 0;
}

//���� ACK Ӧ��
void IIC_Ack(void)
{
    IIC_SCL=0; //ʱ���źŵ͵�ƽ
    SDA_OUT(); //���
    IIC_SDA=0; //�����źŵ͵�ƽ
    delay_us(2);
    IIC_SCL=1; //ʱ���źŸߵ�ƽ
    delay_us(2);
    IIC_SCL=0; //ʱ���źŵ͵�ƽ
}
//������ ACK Ӧ�� 
void IIC_NAck(void)
{
    IIC_SCL=0; //ʱ���źŵ͵�ƽ
    SDA_OUT(); //���
    IIC_SDA=1; //�����źŸߵ�ƽ
    delay_us(2);
    IIC_SCL=1; //ʱ���źŸߵ�ƽ
    delay_us(2);
    IIC_SCL=0; //ʱ���źŵ͵�ƽ
}

//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ�� 
void IIC_Send_Byte(u8 txd)
{ 
    u8 t; 
    SDA_OUT(); 
    IIC_SCL=0; //����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    { 
        IIC_SDA=(txd&0x80)>>7;// ȡ�ֽڵ����λ��ֵ���������� 7 λ
        txd<<=1; //Ȼ�� txd ��������һλ���ȴ���һ��ѭ��ʱ����
        delay_us(2); //�� TEA5767 ��������ʱ���Ǳ����
        IIC_SCL=1; //����һ��ʱ��
        delay_us(2); 
        IIC_SCL=0;
        delay_us(2);
    }
}

//�� 1 ���ֽڣ�ack=1 ʱ������ ACK��ack=0������ nACK 
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN(); //SDA ����Ϊ����
    for(i=0;i<8;i++ )
    {
        IIC_SCL=0; 
        delay_us(2);
        IIC_SCL=1; //����һ��ʱ��
        receive<<=1;//��һ��ѭ�����Ʋ���Ӱ�죬��ΪΪ 0��֮��ÿѭ��һ������һλ
        if(READ_SDA) receive++; //READ_SDA Ϊ��ȡ����ŵĵ�ƽ��Ϊ 1 �Ļ������λ�� 1 
        delay_us(1); 
    }
    if (!ack) IIC_NAck(); //���� nACK
    else IIC_Ack(); //���� ACK 
    return receive;
}
