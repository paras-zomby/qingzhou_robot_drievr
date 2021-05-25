#ifndef __PSTWO_CPP_H__
#define __PSTWO_CPP_H__
#include "delay.h"
#include "sys.h"
#include "debug.h"

#define DI   PCin(2)          //  ����

#define DO_H PCout(1)=1        //����λ��
#define DO_L PCout(1)=0        //����λ��

#define CS_H PCout(3)=1       //CS����
#define CS_L PCout(3)=0       //CS����

#define CLK_H PAout(4)=1      //ʱ������
#define CLK_L PAout(4)=0      //ʱ������

//These are our button constants
#define PSB_L2          0x8000
#define PSB_R2          0x4000
#define PSB_L1          0x2000
#define PSB_R1          0x1000
#define PSB_TRIANGLE    0x0800
#define PSB_CIRCLE      0x0400
#define PSB_CROSS       0x0200
#define PSB_SQUARE      0x0100
#define PSB_SELECT      0x0080
#define PSB_L3          0x0040
#define PSB_R3          0x0020
#define PSB_START       0x0010
#define PSB_PAD_UP      0x0008
#define PSB_PAD_RIGHT   0x0004
#define PSB_PAD_DOWN    0x0002
#define PSB_PAD_LEFT    0x0001

#define PSB_GREEN       0x0800
#define PSB_RED         0x0400
#define PSB_BLUE        0x0200
#define PSB_PINK        0x0100

//#define WHAMMY_BAR        8

//These are stick values
#define PSS_RX 5                //��ҡ��X������
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8


class CPS2
{
public:
    static const u16 MASK[16];
    static const u8 Comd[2];   //��ʼ�����������
    static bool is_unique;
    CDebug* const debug;
private:
    u8 Data[9];
    u16 HandKey;
    void PS2_ShortPoll(void);
    void PS2_Cmd(u8 CMD);   //���ֱ���������
    void PS2_EnterConfing(void);	 //��������
    void PS2_TurnOnAnalogMode(void); //����ģ����
    void PS2_VibrationMode(void);    //������
    void PS2_ExitConfing(void);	     //�������
    void PS2_SetInit(void);		     //���ó�ʼ��
    void PS2_GPIO_Config(void);      //����PS2��GPIO�˿�
    void PS2_ClearData(void);	  //������ݻ�����
    void PS2_Vibration(u8 motor1, u8 motor2);//������motor1  0xFF���������أ�motor2  0x40~0xFF
    bool PS2_RedLight(void);   //�ж��Ƿ�Ϊ���ģʽ
public:
    CPS2(CDebug* const _debug);
    ~CPS2();
    void PS2_ReadData(void); //���ֱ�����
    u16 PS2_ReturnPressedKey(void);   //����ֵ��ȡ
    u8 PS2_AnologData(u8 button); //�õ�һ��ҡ�˵�ģ����
};

#endif //__PSTWO_CPP_H__





