#include "imu-cpp.h"


//IIC ʹ�õ�ģʽ
//#define USE_HARDWARE_IIC  //ʹ��Ӳ��IIC(IIC����)������ʹ�����ģ��IIC

//��imu.cpp��ʹ�õĺ궨��
#define PRINT_DEBUG_INFO(...) {char* pstr=NULL;asprintf(&pstr,__VA_ARGS__);debug->InfoHandle("IMU",pstr);free(pstr);}
#define PRINT_ERROR_INFO(...) {char* pstr=NULL;asprintf(&pstr,__VA_ARGS__);debug->ErrorHandle("IMU",pstr);free(pstr);}


bool CImu::is_unique = 1;

CImu::CImu(CDebug* const _debug)
    :debug(_debug)
{
    if (is_unique)
    {
        timeout = 300;
        is_unique = 0;
#ifndef USE_HARDWARE_IIC
        IIC_Init();
#else
        I2C2_Init();
#endif //USE_HARDWARE_IIC
    
        if(MPU9250_Read_Reg(GYRO_ADDRESS,WHO_AM_I)==0x71)
        {
            MPU9250_Write_Reg(GYRO_ADDRESS,PWR_MGMT_1,0x00); //�������״̬
            MPU9250_Write_Reg(GYRO_ADDRESS,SMPLRT_DIV,0x07); //����Ƶ�� 125Hz
            MPU9250_Write_Reg(GYRO_ADDRESS,CONFIG,0X06); //��ͨ�˲��� 5Hz
            MPU9250_Write_Reg(GYRO_ADDRESS,GYRO_CONFIG,0X18); //����������,���� 2000 ��
            MPU9250_Write_Reg(GYRO_ADDRESS,ACCEL_CONFIG,0X18); //���ٶ�����,���� 16g
        }
        else
            PRINT_ERROR_INFO("IMU Init Failed!");
    }
    else
    {
        PRINT_ERROR_INFO("class has been redefined")
    }
}

CImu::~CImu()
{
    is_unique = 1;
}

const CImu::Accel_Data& CImu::Get_Accel()
{
    return _accel;
}
const CImu::Gyro_Data& CImu::Get_Gyro()
{
    return _gyro;
}
const CImu::Mag_Data& CImu::Get_Mag()
{
    return _mag;
}

void CImu::ShowData_OLED()
{
    debug->OLED_ShowString(00,00,"AcX:");debug->OLED_ShowNumber(15,00,ABS(_accel.accelX)*160/32767,5,12);
    debug->OLED_ShowString(00,10,"AcY:");debug->OLED_ShowNumber(15,10,ABS(_accel.accelY)*160/32767,5,12);
    debug->OLED_ShowString(00,20,"AcZ:");debug->OLED_ShowNumber(15,20,ABS(_accel.accelZ)*160/32767,5,12);
    debug->OLED_ShowString(00,30,"GyX:");debug->OLED_ShowNumber(15,30,ABS(_gyro.gyroX)*200/32767,5,12);
    debug->OLED_ShowString(00,40,"GyY:");debug->OLED_ShowNumber(15,40,ABS(_gyro.gyroY)*200/32767,5,12);
    debug->OLED_ShowString(00,50,"GyZ:");debug->OLED_ShowNumber(15,50,ABS(_gyro.gyroZ)*200/32767,5,12);
    debug->OLED_ShowString(80,00,"MaX:");debug->OLED_ShowNumber(95,00,ABS(_mag.magX)/20,5,12);
    debug->OLED_ShowString(80,10,"MaY:");debug->OLED_ShowNumber(95,10,ABS(_mag.magY)/20,5,12);
    debug->OLED_ShowString(80,20,"MaZ:");debug->OLED_ShowNumber(95,20,ABS(_mag.magZ)/20,5,12);

    debug->OLED_Refresh_Gram();    //ˢ��
}

//�ɹ�����SUCCESS��ʧ�ܿ���
ErrorStatus CImu::MPU9250_Write_Reg(u8 Slave_add,u8 reg_add,u8 reg_dat)
{
    ErrorStatus ret = SUCCESS;
#ifndef USE_HARDWARE_IIC
    IIC_Start(); //��ʼ
    IIC_Send_Byte(Slave_add); //���� I2C д��ַ
    IIC_Wait_Ack(); //��Ӧ
    IIC_Send_Byte(reg_add); //���ͼĴ�����ַ
    IIC_Wait_Ack(); //��Ӧ
    IIC_Send_Byte(reg_dat); //��������
    IIC_Wait_Ack(); //��Ӧ
    IIC_Stop(); //ֹͣ
#else
    I2C2_START//������ʼ�źţ����EV5
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_MODE_SELECT, timeout);
    I2C2_SENDADDR_W(Slave_add)//���ʹӻ���ַ�����EV6
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout);
    I2C2_SENDDATA(reg_add)//���͵�һ�����ݣ�д���ַ�������EV8
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_BYTE_TRANSMITTING, timeout);
    I2C2_SENDDATA(reg_dat)//�������һ�����ݣ�д����Ϣ�������EV8_2
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);
    I2C2_STOP//����ֹͣ�ź�
    if (ret == ERROR) PRINT_ERROR_INFO("Write Reg Error");
#endif //USE_HARDWARE_IIC
    return ret;
}

//�ɹ�������ֵ��ʧ�ܿ���
u8 CImu::MPU9250_Read_Reg(u8 Slave_add,u8 reg_add)
{
    ErrorStatus ret = SUCCESS;
    u8 data;
#ifndef USE_HARDWARE_IIC
    IIC_Start(); //��ʼ
    IIC_Send_Byte(Slave_add); //���� I2C д��ַ
    data=IIC_Wait_Ack(); //��Ӧ
    IIC_Send_Byte(reg_add); //���ͼĴ�����ַ
    data=IIC_Wait_Ack(); //��Ӧ
    IIC_Start(); //�ڴ˷��Ϳ�ʼ�ź�
    IIC_Send_Byte(Slave_add+1); //���� I2C ����ַ
    data=IIC_Wait_Ack(); //��Ӧ
    data=IIC_Read_Byte(0); //��ȡ 1 �ֽ�
    IIC_Stop();
#else
    I2C2_START//������ʼ�źţ����EV5
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_MODE_SELECT, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read Reg Error 1");
    I2C2_SENDADDR_W(Slave_add)//���ʹӻ���ַ�����EV6
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 2");
    I2C2_SENDDATA(reg_add)//����Ψһ���ݣ���ȡ��ַ�������EV8--������
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_BYTE_TRANSMITTING, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 3");
    I2C2_START//������ʼ�źţ����EV5
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_MODE_SELECT, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 4");
    I2C2_SENDADDR_R(Slave_add)//���ʹӻ���ַ�����EV6
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 5");
    data = I2C2_RECEIVEDATA//�������ݣ����EV7
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_BYTE_RECEIVED, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 6");
    I2C2_STOP//����ֹͣ�ź�
#endif
    return (ret==ERROR)?0xFF:data;
}

//��ȡ���ٶ����ݵĺ���
void CImu::MPU9250_READ_ACCEL()
{ 
    u8 BUF[6];
    BUF[0]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_XOUT_L); //�� X ���ٶȵ��ֽ�
    BUF[1]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_XOUT_H); //�� X ���ٶȸ��ֽ�
    _accel.accelX=(BUF[1]<<8)|BUF[0];
    BUF[2]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_YOUT_L); //�� Y ���ٶȵ��ֽ�
    BUF[3]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_YOUT_H); //�� Y ���ٶȸ��ֽ�
    _accel.accelY=(BUF[3]<<8)|BUF[2];
    BUF[4]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_ZOUT_L); //�� Z ���ٶȵ��ֽ�
    BUF[5]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_ZOUT_H); //�� Z ���ٶȸ��ֽ�
    _accel.accelZ=(BUF[5]<<8)|BUF[4];
}
//��ȡ���ٶ����ݵĺ���
void CImu::MPU9250_READ_GYRO()
{
    u8 BUF[8];
    BUF[0]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_XOUT_L); //�� X ���ٶȵ��ֽ�
    BUF[1]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_XOUT_H); //�� X ���ٶȸ��ֽ�
    _gyro.gyroX=(BUF[1]<<8)|BUF[0];
    BUF[2]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_YOUT_L); //�� Y ���ٶȵ��ֽ�
    BUF[3]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_YOUT_H); //�� Y ���ٶȸ��ֽ�
    _gyro.gyroY=(BUF[3]<<8)|BUF[2];
    BUF[4]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_ZOUT_L); //�� Z ���ٶȵ��ֽ�
    BUF[5]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_ZOUT_H); //�� Z ���ٶȸ��ֽ�
    _gyro.gyroZ=(BUF[5]<<8)|BUF[4];
}
//��ȡ���������ݵĺ���
void CImu::MPU9250_READ_MAG() 
{ 
    u8 BUF[6];
    MPU9250_Write_Reg(GYRO_ADDRESS,INT_PIN_CFG,0x02); //turn on Bypass Mode 
    delay_ms(10);
    MPU9250_Write_Reg(MAG_ADDRESS,0x0A,0x01); //������������ת��,�����������������ݲ���
    delay_ms(10);
    BUF[0]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_XOUT_L); //�� X �����Ƶ��ֽ�
    BUF[1]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_XOUT_H); //�� X �����Ƹ��ֽ�
    _mag.magX=(BUF[1]<<8)|BUF[0];
    BUF[2]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_YOUT_L); //�� Y �����Ƶ��ֽ�
    BUF[3]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_YOUT_H); //�� Y �����Ƹ��ֽ�
    _mag.magY=(BUF[3]<<8)|BUF[2];
    BUF[4]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_ZOUT_L); //�� Z �����Ƶ��ֽ�
    BUF[5]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_ZOUT_H); //�� Z �����Ƹ��ֽ�
    _mag.magZ=(BUF[5]<<8)|BUF[4];
}
void CImu::ReadData() //��ȡ IMU ���ݺ���
{
    MPU9250_READ_ACCEL(); //��ȡ���ٶ�
    MPU9250_READ_GYRO(); //��ȡ���ٶ�
    MPU9250_READ_MAG(); //��ȡ������
}

