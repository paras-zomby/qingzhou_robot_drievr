#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f10x.h"

#include "iic.h"
#include "delay.h"
#include "debug.h"
#include "sim-iic.h"

/************** I2C Address *****************/
//AD0=0,IIC ��ַ 1101000�����λ��� 0 �� 1,��ʾд���߶�;
//����д��ַ��:11010000 (0xD0);����ַ��:11010001 (0xD1)
//�����Ƶ�ַΪ:11000(0x18)
#define ACCEL_ADDRESS 0xD0 //���ٶ� I2C ��ַ
#define GYRO_ADDRESS 0xD0 //���ٶ� I2C ��ַ
#define MAG_ADDRESS 0x18 //������ I2C ��ַ
#define SMPLRT_DIV 0X19 //�����ǲ����� ����ֵΪ 0x07 1000/(1+7)=125 Hz
#define CONFIG 0X1A //��ͨ�˲��� ����ֵΪ 0x06 5Hz
#define GYRO_CONFIG 0X1B //�����ǲ�����Χ 0x18 ���� 2000 ��
#define ACCEL_CONFIG 0X1C //���ٶȼƲ�����Χ 0x18 ���� 16g
#define ACCEL_CONFIG2 0X1D //���ٶȼƵ�ͨ�˲��� 0x06 5Hz
//���ٶ�������ݼĴ�����ַ
#define ACCEL_XOUT_H 0X3B
#define ACCEL_XOUT_L 0X3C
#define ACCEL_YOUT_H 0X3D
#define ACCEL_YOUT_L 0X3E
#define ACCEL_ZOUT_H 0X3F
#define ACCEL_ZOUT_L 0X40
//������������ݼĴ�����ַ
#define GYRO_XOUT_H 0X43
#define GYRO_XOUT_L 0X44
#define GYRO_YOUT_H 0X45
#define GYRO_YOUT_L 0X46
#define GYRO_ZOUT_H 0X47
#define GYRO_ZOUT_L 0X48
//������������ݼĴ�����ַ
#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08
//�����Ĵ�����ַ
#define WHO_AM_I 0X75
#define PWR_MGMT_1 0X6B
#define INT_PIN_CFG 0X37


class CImu
{
private:
    static bool is_unique;
public:
//    struct Accel_Data
//    {
//        short accelX,accelY,accelZ; //��������ٶȼ�
//    };
//    struct Gyro_Data
//    {
//        short gyroX,gyroY,gyroZ; //������������
//    };
//    struct Mag_Data
//    {
//        short magX,magY,magZ; //�����������
//    };
    
private:
    CDebug* const debug;
    u16 timeout;

    float data[9];

//    Accel_Data _accel;//���ٶȼ�
//    Gyro_Data _gyro; // ������
//    Mag_Data _mag; // ������

    ErrorStatus MPU9250_Write_Reg(u8 Slave_add,u8 reg_add,u8 reg_dat); //д�Ĵ���
    u8 MPU9250_Read_Reg(u8 Slave_add,u8 reg_add); //���Ĵ���
    void MPU9250_READ_ACCEL(); //�����ٶ�
    void MPU9250_READ_GYRO(); //�����ٶ�
    void MPU9250_READ_MAG(); //��������

public:
    CImu(CDebug* const _debug);
    ~CImu();
    
    const float* ReadData(void); //�� imu
    void ShowData_OLED(void); //��������ʾ��OLED

//    const CImu::Accel_Data& Get_Accel(void);//��ȡAccel����
//    const CImu::Gyro_Data& Get_Gyro(void);//��ȡGyro����
//    const CImu::Mag_Data& Get_Mag(void);//��ȡMag����
};

#endif  //__IMU_H__
