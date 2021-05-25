#include "imu-c.h"

short Accel[3]; // ���ٶȼ�
short Gyro[3]; // ������
short Mag[3]; // ������
short gyroX,gyroY,gyroZ; //������������
short accelX,accelY,accelZ; //��������ٶȼ�
short magX,magY,magZ; //�����������


// I2C д�����ݵ� 4 ��д�Ĵ�����ʱ�򷽷���ʵ��дһ���ֽڵ��ӻ�
void MPU9250_Write_Reg(u8 Slave_add,u8 reg_add,u8 reg_dat)
{
IIC_Start(); //��ʼ
IIC_Send_Byte(Slave_add); //���� I2C д��ַ
IIC_Wait_Ack(); //��Ӧ
IIC_Send_Byte(reg_add); //���ͼĴ�����ַ
IIC_Wait_Ack(); //��Ӧ
IIC_Send_Byte(reg_dat); //��������
IIC_Wait_Ack(); //��Ӧ
IIC_Stop(); //ֹͣ
}
//I2C �������ݵ� 4 �ڶȼĴ�����ʱ�򷽷���ʵ�ֶ�һ���ֽ�
u8 MPU9250_Read_Reg(u8 Slave_add,u8 reg_add)
{
u8 temp=0;
IIC_Start(); //��ʼ
IIC_Send_Byte(Slave_add); //���� I2C д��ַ
temp=IIC_Wait_Ack(); //��Ӧ
IIC_Send_Byte(reg_add); //���ͼĴ�����ַ
temp=IIC_Wait_Ack(); //��Ӧ
IIC_Start(); //�ڴ˷��Ϳ�ʼ�ź�
IIC_Send_Byte(Slave_add+1); //���� I2C ����ַ
temp=IIC_Wait_Ack(); //��Ӧ
temp=IIC_Read_Byte(0); //��ȡ 1 �ֽ�
IIC_Stop();
return temp;
}
// ��ʼ��
u8 MPU9250_Init(void)
{
IIC_Init();
if(MPU9250_Read_Reg(GYRO_ADDRESS,WHO_AM_I)==0x71)
{
MPU9250_Write_Reg(GYRO_ADDRESS,PWR_MGMT_1,0x00); //�������״̬
MPU9250_Write_Reg(GYRO_ADDRESS,SMPLRT_DIV,0x07); //����Ƶ�� 125Hz
MPU9250_Write_Reg(GYRO_ADDRESS,CONFIG,0X06); //��ͨ�˲��� 5Hz
MPU9250_Write_Reg(GYRO_ADDRESS,GYRO_CONFIG,0X18); //����������,���� 2000 ��
MPU9250_Write_Reg(GYRO_ADDRESS,ACCEL_CONFIG,0X18); //���ٶ�����,���� 16g
return 0;
}
return 1;
}
//��ȡ���ٶ����ݵĺ���
void MPU9250_READ_ACCEL(short *accData)
{ 
u8 BUF[6];
BUF[0]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_XOUT_L); //�� X ���ٶȵ��ֽ�
BUF[1]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_XOUT_H); //�� X ���ٶȸ��ֽ�
accelX=(BUF[1]<<8)|BUF[0];
BUF[2]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_YOUT_L); //�� Y ���ٶȵ��ֽ�
BUF[3]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_YOUT_H); //�� Y ���ٶȸ��ֽ�
accelY=(BUF[3]<<8)|BUF[2];
BUF[4]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_ZOUT_L); //�� Z ���ٶȵ��ֽ�
BUF[5]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_ZOUT_H); //�� Z ���ٶȸ��ֽ�
 accelZ=(BUF[5]<<8)|BUF[4];
}
//��ȡ���ٶ����ݵĺ���
void MPU9250_READ_GYRO(short *gyroData)
{
u8 BUF[8];
BUF[0]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_XOUT_L); //�� X ���ٶȵ��ֽ�
BUF[1]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_XOUT_H); //�� X ���ٶȸ��ֽ�
gyroX=(BUF[1]<<8)|BUF[0];
BUF[2]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_YOUT_L); //�� Y ���ٶȵ��ֽ�
BUF[3]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_YOUT_H); //�� Y ���ٶȸ��ֽ�
gyroY=(BUF[3]<<8)|BUF[2];
BUF[4]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_ZOUT_L); //�� Z ���ٶȵ��ֽ�
BUF[5]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_ZOUT_H); //�� Z ���ٶȸ��ֽ�
gyroZ=(BUF[5]<<8)|BUF[4];
}
//��ȡ���������ݵĺ���
void MPU9250_READ_MAG(short *magData) 
{ 
u8 BUF[6];
MPU9250_Write_Reg(GYRO_ADDRESS,INT_PIN_CFG,0x02); //turn on Bypass Mode 
delay_ms(10);
MPU9250_Write_Reg(MAG_ADDRESS,0x0A,0x01); //������������ת��,�����������������ݲ���
delay_ms(10);
BUF[0]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_XOUT_L); //�� X �����Ƶ��ֽ�
BUF[1]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_XOUT_H); //�� X �����Ƹ��ֽ�
magX=(BUF[1]<<8)|BUF[0];
BUF[2]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_YOUT_L); //�� Y �����Ƶ��ֽ�
BUF[3]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_YOUT_H); //�� Y �����Ƹ��ֽ�
magY=(BUF[3]<<8)|BUF[2];
BUF[4]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_ZOUT_L); //�� Z �����Ƶ��ֽ�
BUF[5]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_ZOUT_H); //�� Z �����Ƹ��ֽ�
magZ=(BUF[5]<<8)|BUF[4];
}
void readImu() //��ȡ IMU ���ݺ���
{
MPU9250_READ_ACCEL(Accel); //��ȡ���ٶ�
MPU9250_READ_GYRO(Gyro); //��ȡ���ٶ�
MPU9250_READ_MAG(Mag); //��ȡ������
}
