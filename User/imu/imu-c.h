#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f10x.h"

#include "iic.h"
#include "sys.h"
#include "delay.h"

/************** I2C Address *****************/
//AD0=0,IIC 地址 1101000，最低位添加 0 或 1,表示写或者读;
//所以写地址是:11010000 (0xD0);读地址是:11010001 (0xD1)
//磁力计地址为:11000(0x18)
#define ACCEL_ADDRESS 0xD0 //加速度 I2C 地址
#define GYRO_ADDRESS 0xD0 //角速度 I2C 地址
#define MAG_ADDRESS 0x18 //磁力计 I2C 地址
#define SMPLRT_DIV 0X19 //陀螺仪采样率 典型值为 0x07 1000/(1+7)=125 Hz
#define CONFIG 0X1A //低通滤波器 典型值为 0x06 5Hz
#define GYRO_CONFIG 0X1B //陀螺仪测量范围 0x18 正负 2000 度
#define ACCEL_CONFIG 0X1C //加速度计测量范围 0x18 正负 16g
#define ACCEL_CONFIG2 0X1D //加速度计低通滤波器 0x06 5Hz
//加速度输出数据寄存器地址
#define ACCEL_XOUT_H 0X3B
#define ACCEL_XOUT_L 0X3C
#define ACCEL_YOUT_H 0X3D
#define ACCEL_YOUT_L 0X3E
#define ACCEL_ZOUT_H 0X3F
#define ACCEL_ZOUT_L 0X40
//陀螺仪输出数据寄存器地址
#define GYRO_XOUT_H 0X43
#define GYRO_XOUT_L 0X44
#define GYRO_YOUT_H 0X45
#define GYRO_YOUT_L 0X46
#define GYRO_ZOUT_H 0X47
#define GYRO_ZOUT_L 0X48
//磁力计输出数据寄存器地址
#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08
//其他寄存器地址
#define WHO_AM_I 0X75
#define PWR_MGMT_1 0X6B
#define INT_PIN_CFG 0X37

void MPU9250_Write_Reg(u8 Slave_add,u8 reg_add,u8 reg_dat); //写寄存器
u8 MPU9250_Read_Reg(u8 Slave_add,u8 reg_add); //读寄存器
void MPU6050_ReadData(u8 Slave_add,u8 reg_add,u8*Read,u8 num); //读数据
u8 MPU9250_Init(void); //初始化
void MPU9250_READ_ACCEL(short *accData); //读加速度
void MPU9250_READ_GYRO(short *gyroData); //读角速度
void MPU9250_READ_MAG(short *magData); //读磁力计
void readImu(void); //读 imu

#endif  //__IMU_H__
