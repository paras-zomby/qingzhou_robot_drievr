#ifndef __PSTWO_CPP_H__
#define __PSTWO_CPP_H__
#include "delay.h"
#include "sys.h"
#include "debug.h"

#define DI   PCin(2)          //  输入

#define DO_H PCout(1)=1        //命令位高
#define DO_L PCout(1)=0        //命令位低

#define CS_H PCout(3)=1       //CS拉高
#define CS_L PCout(3)=0       //CS拉低

#define CLK_H PAout(4)=1      //时钟拉高
#define CLK_L PAout(4)=0      //时钟拉低

//These are our button constants
//#define PSB_L2          0x8000
//#define PSB_R2          0x4000
//#define PSB_L1          0x2000
//#define PSB_R1          0x1000
//#define PSB_TRIANGLE    0x0800
//#define PSB_CIRCLE      0x0400
//#define PSB_CROSS       0x0200
//#define PSB_SQUARE      0x0100
//#define PSB_SELECT      0x0080
//#define PSB_L3          0x0040
//#define PSB_R3          0x0020
//#define PSB_START       0x0010
//#define PSB_PAD_UP      0x0008
//#define PSB_PAD_RIGHT   0x0004
//#define PSB_PAD_DOWN    0x0002
//#define PSB_PAD_LEFT    0x0001

#define PSB_GREEN       PSB_TRIANGLE
#define PSB_RED         PSB_CIRCLE
#define PSB_BLUE        PSB_CROSS
#define PSB_PINK        PSB_SQUARE

//#define WHAMMY_BAR        8

//These are stick values
//#define PSS_RX 5                //右摇杆X轴数据
//#define PSS_RY 6
//#define PSS_LX 7
//#define PSS_LY 8

enum class PS2_POLL:unsigned char{
    PSS_RX = 5,
    PSS_RY = 6,
    PSS_LX = 7,
    PSS_LY = 8
};

enum class PS2_KEY:unsigned short{
    PSB_L2      =   0x8000,
    PSB_R2      =   0x4000,
    PSB_L1      =   0x2000,
    PSB_R1      =   0x1000,
    PSB_TRIANGLE=   0x0800,
    PSB_CIRCLE  =   0x0400,
    PSB_CROSS   =   0x0200,
    PSB_SQUARE  =   0x0100,
    PSB_SELECT  =   0x0080,
    PSB_L3      =   0x0040,
    PSB_R3      =   0x0020,
    PSB_START   =   0x0010,
    PSB_PAD_UP  =   0x0008,
    PSB_PAD_RIGHT=  0x0004,
    PSB_PAD_DOWN=   0x0002,
    PSB_PAD_LEFT=   0x0001
};

class CPS2
{
public:
    static const u8 Comd[2];   //开始命令。请求数据
    static bool is_unique;
    CDebug* const debug;
private:
    u8 Data[9];
    u16 HandKey;
    void PS2_ShortPoll(void);
    void PS2_Cmd(u8 CMD);   //向手柄发送命令
    void PS2_EnterConfing(void);	 //进入配置
    void PS2_TurnOnAnalogMode(void); //发送模拟量
    void PS2_VibrationMode(void);    //振动设置
    void PS2_ExitConfing(void);	     //完成配置
    void PS2_SetInit(void);		     //配置初始化
    void PS2_GPIO_Config(void);      //配置PS2的GPIO端口
    void PS2_ClearData(void);	  //清除数据缓冲区
    void PS2_Vibration(u8 motor1, u8 motor2);//振动设置motor1  0xFF开，其他关，motor2  0x40~0xFF
    bool PS2_RedLight(void);   //判断是否为红灯模式
public:
    CPS2(CDebug* const _debug);
    ~CPS2();
    void PS2_ReadData(void); //读手柄数据
    bool PS2_IfKeyBnClicked(enum PS2_KEY _key);   //指定按键是否按下
    u8 PS2_AnologData(enum PS2_POLL _poll); //得到一个摇杆的模拟量
};

#endif //__PSTWO_CPP_H__





