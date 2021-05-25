#include "imu-cpp.h"


//IIC 使用的模式
//#define USE_HARDWARE_IIC  //使用硬件IIC(IIC外设)，否则使用软件模拟IIC

//在imu.cpp中使用的宏定义
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
            MPU9250_Write_Reg(GYRO_ADDRESS,PWR_MGMT_1,0x00); //解除休眠状态
            MPU9250_Write_Reg(GYRO_ADDRESS,SMPLRT_DIV,0x07); //采样频率 125Hz
            MPU9250_Write_Reg(GYRO_ADDRESS,CONFIG,0X06); //低通滤波器 5Hz
            MPU9250_Write_Reg(GYRO_ADDRESS,GYRO_CONFIG,0X18); //陀螺仪量程,正负 2000 度
            MPU9250_Write_Reg(GYRO_ADDRESS,ACCEL_CONFIG,0X18); //加速度量程,正负 16g
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

    debug->OLED_Refresh_Gram();    //刷新
}

//成功返回SUCCESS，失败卡死
ErrorStatus CImu::MPU9250_Write_Reg(u8 Slave_add,u8 reg_add,u8 reg_dat)
{
    ErrorStatus ret = SUCCESS;
#ifndef USE_HARDWARE_IIC
    IIC_Start(); //开始
    IIC_Send_Byte(Slave_add); //发送 I2C 写地址
    IIC_Wait_Ack(); //响应
    IIC_Send_Byte(reg_add); //发送寄存器地址
    IIC_Wait_Ack(); //响应
    IIC_Send_Byte(reg_dat); //发送数据
    IIC_Wait_Ack(); //响应
    IIC_Stop(); //停止
#else
    I2C2_START//发送起始信号，检测EV5
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_MODE_SELECT, timeout);
    I2C2_SENDADDR_W(Slave_add)//发送从机地址，检测EV6
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout);
    I2C2_SENDDATA(reg_add)//发送第一个数据（写入地址），检测EV8
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_BYTE_TRANSMITTING, timeout);
    I2C2_SENDDATA(reg_dat)//发送最后一个数据（写入信息），检测EV8_2
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);
    I2C2_STOP//发送停止信号
    if (ret == ERROR) PRINT_ERROR_INFO("Write Reg Error");
#endif //USE_HARDWARE_IIC
    return ret;
}

//成功返回数值，失败卡死
u8 CImu::MPU9250_Read_Reg(u8 Slave_add,u8 reg_add)
{
    ErrorStatus ret = SUCCESS;
    u8 data;
#ifndef USE_HARDWARE_IIC
    IIC_Start(); //开始
    IIC_Send_Byte(Slave_add); //发送 I2C 写地址
    data=IIC_Wait_Ack(); //响应
    IIC_Send_Byte(reg_add); //发送寄存器地址
    data=IIC_Wait_Ack(); //响应
    IIC_Start(); //在此发送开始信号
    IIC_Send_Byte(Slave_add+1); //发送 I2C 读地址
    data=IIC_Wait_Ack(); //响应
    data=IIC_Read_Byte(0); //读取 1 字节
    IIC_Stop();
#else
    I2C2_START//发送起始信号，检测EV5
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_MODE_SELECT, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read Reg Error 1");
    I2C2_SENDADDR_W(Slave_add)//发送从机地址，检测EV6
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 2");
    I2C2_SENDDATA(reg_add)//发送唯一数据（读取地址），检测EV8--？？？
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_BYTE_TRANSMITTING, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 3");
    I2C2_START//发送起始信号，检测EV5
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_MODE_SELECT, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 4");
    I2C2_SENDADDR_R(Slave_add)//发送从机地址，检测EV6
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 5");
    data = I2C2_RECEIVEDATA//接收数据，检测EV7
    ret = I2C2_WaitForSuc(I2C_EVENT_MASTER_BYTE_RECEIVED, timeout);
    if(ret == ERROR) PRINT_ERROR_INFO("Read RegError 6");
    I2C2_STOP//发送停止信号
#endif
    return (ret==ERROR)?0xFF:data;
}

//读取加速度数据的函数
void CImu::MPU9250_READ_ACCEL()
{ 
    u8 BUF[6];
    BUF[0]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_XOUT_L); //读 X 加速度低字节
    BUF[1]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_XOUT_H); //读 X 加速度高字节
    _accel.accelX=(BUF[1]<<8)|BUF[0];
    BUF[2]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_YOUT_L); //读 Y 加速度低字节
    BUF[3]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_YOUT_H); //读 Y 加速度高字节
    _accel.accelY=(BUF[3]<<8)|BUF[2];
    BUF[4]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_ZOUT_L); //读 Z 加速度低字节
    BUF[5]=MPU9250_Read_Reg(ACCEL_ADDRESS,ACCEL_ZOUT_H); //读 Z 加速度高字节
    _accel.accelZ=(BUF[5]<<8)|BUF[4];
}
//读取角速度数据的函数
void CImu::MPU9250_READ_GYRO()
{
    u8 BUF[8];
    BUF[0]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_XOUT_L); //读 X 角速度低字节
    BUF[1]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_XOUT_H); //读 X 角速度高字节
    _gyro.gyroX=(BUF[1]<<8)|BUF[0];
    BUF[2]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_YOUT_L); //读 Y 角速度低字节
    BUF[3]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_YOUT_H); //读 Y 角速度高字节
    _gyro.gyroY=(BUF[3]<<8)|BUF[2];
    BUF[4]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_ZOUT_L); //读 Z 角速度低字节
    BUF[5]=MPU9250_Read_Reg(GYRO_ADDRESS,GYRO_ZOUT_H); //读 Z 角速度高字节
    _gyro.gyroZ=(BUF[5]<<8)|BUF[4];
}
//读取磁力计数据的函数
void CImu::MPU9250_READ_MAG() 
{ 
    u8 BUF[6];
    MPU9250_Write_Reg(GYRO_ADDRESS,INT_PIN_CFG,0x02); //turn on Bypass Mode 
    delay_ms(10);
    MPU9250_Write_Reg(MAG_ADDRESS,0x0A,0x01); //用来启动单次转换,否则磁力计输出的数据不变
    delay_ms(10);
    BUF[0]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_XOUT_L); //读 X 磁力计低字节
    BUF[1]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_XOUT_H); //读 X 磁力计高字节
    _mag.magX=(BUF[1]<<8)|BUF[0];
    BUF[2]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_YOUT_L); //读 Y 磁力计低字节
    BUF[3]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_YOUT_H); //读 Y 磁力计高字节
    _mag.magY=(BUF[3]<<8)|BUF[2];
    BUF[4]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_ZOUT_L); //读 Z 磁力计低字节
    BUF[5]=MPU9250_Read_Reg(MAG_ADDRESS,MAG_ZOUT_H); //读 Z 磁力计高字节
    _mag.magZ=(BUF[5]<<8)|BUF[4];
}
void CImu::ReadData() //读取 IMU 数据函数
{
    MPU9250_READ_ACCEL(); //读取加速度
    MPU9250_READ_GYRO(); //读取角速度
    MPU9250_READ_MAG(); //读取磁力计
}

