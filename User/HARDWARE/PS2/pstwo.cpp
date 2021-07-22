#include "pstwo-cpp.h"

#define DELAY_TIME  delay_us(5);

bool CPS2::is_unique = 1;
const u8 CPS2::Comd[2]={0x01,0x42};	//开始命令。请求数据

CPS2::CPS2(CDebug* const _debug)
    :debug(_debug)
{
    if (is_unique)
    {
        PS2_GPIO_Config();
        PS2_SetInit();
        is_unique = 0;
    }
    else
    {
        debug->ErrorHandle("PS2", "class has been redefined");
    }
}

CPS2::~CPS2()
{
    is_unique = 1;
}

void CPS2::PS2_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE); //使能端口时钟
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	            //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);					      //根据设定参数初始化GPIO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_3;    //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //50M
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  //根据设定参数初始化GPIOA

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4;                //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //50M
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  //根据设定参数初始化GPIOB
}


//向手柄发送命令
void CPS2::PS2_Cmd(u8 CMD)
{
    volatile u16 ref=0x01;
    Data[1] = 0;
    for(ref=0x01;ref<0x0100;ref<<=1)
    {
        if(ref&CMD) DO_H;             //输出一位控制位
        else DO_L;

        CLK_H;                        //时钟拉高
        DELAY_TIME;
        CLK_L;
        DELAY_TIME;
        CLK_H;
        if(DI) Data[1] = ref|Data[1];
    }
    delay_us(16);
}
//判断是否为红灯模式,0x41=模拟绿灯，0x73=模拟红灯
//返回值；1，红灯模式
//        其他，其他模式
bool CPS2::PS2_RedLight(void)
{
    CS_L;
    PS2_Cmd(Comd[0]);  //开始命令
    PS2_Cmd(Comd[1]);  //请求数据
    CS_H;
    if( Data[1] == 0X73)   return 1 ;
    else return 0;
}
//读取手柄数据
void CPS2::PS2_ReadData(void)
{
    PS2_ClearData();
    volatile u8 byte=0;
    volatile u16 ref=0x01;
    CS_L;
    PS2_Cmd(Comd[0]);  //开始命令
    PS2_Cmd(Comd[1]);  //请求数据
    for(byte=2;byte<9;++byte)          //开始接受数据
    {
        for(ref=0x01;ref<0x100;ref<<=1)
        {
            CLK_H;
            DELAY_TIME;
            CLK_L;
            DELAY_TIME;
            CLK_H;
              if(DI)
              Data[byte] = ref|Data[byte];
        }
        delay_us(16);
    }
    CS_H;
    HandKey=~((Data[3]<<8)|Data[4]); //这是16个按键，用与来存储在无符号16位数中
                                     //其中，原数据按下的按键对应位是0，没按下的按键对应位是1。
                                     //通过取反操作，变成按下1松开0。
}

//对按键进行处理，输入参数是想要检测的按键，可以！按位与！
//所有输入按键只要有一个按下就会返回真
//只检测上升沿，而且同时输入的按键分开计算第一次按下
bool CPS2::PS2_IfKeyBnClicked(enum PS2_KEY _key)
{
    u16 key = (u16)_key;
    static u16 pressed_key = 0;
    if((HandKey & key) && !(pressed_key & key))
    {
        pressed_key |= (key & HandKey);
        return true;
    }
    else
    {
        pressed_key &= HandKey;
        return false;
    }
}

bool CPS2::PS2_IfKeyBnPressed(enum PS2_KEY _key)
{
    return (HandKey & (u16)_key);
}

//得到一个摇杆的模拟量  范围0~256
u8 CPS2::PS2_AnologData(enum PS2_POLL _poll)
{
    return Data[(u8)_poll];
}

//清除数据缓冲区
void CPS2::PS2_ClearData()
{
    u8 a;
    for(a=0;a<9;a++)
        Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: 手柄震动函数，
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:右侧小震动电机 0x00关，其他开
	   motor2:左侧大震动电机 0x40~0xFF 电机开，值越大 震动越大
******************************************************/
void CPS2::PS2_Vibration(u8 motor1, u8 motor2)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  //开始命令
    PS2_Cmd(0x42);  //请求数据
    PS2_Cmd(0X00);
    PS2_Cmd(motor1);
    PS2_Cmd(motor2);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    CS_H;
    delay_us(16);  
}
//short poll
void CPS2::PS2_ShortPoll(void)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  
    PS2_Cmd(0x42);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    CS_H;
    delay_us(16);
}
//进入配置
void CPS2::PS2_EnterConfing(void)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  
    PS2_Cmd(0x43);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x01);
    PS2_Cmd(0x00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    CS_H;
    delay_us(16);
}
//发送模式设置
void CPS2::PS2_TurnOnAnalogMode(void)
{
    CS_L;
    PS2_Cmd(0x01);  
    PS2_Cmd(0x44);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x01); //analog=0x01;digital=0x00  软件设置发送模式
    PS2_Cmd(0x03); //Ox03锁存设置，即不可通过按键“MODE”设置模式。
                   //0xEE不锁存软件设置，可通过按键“MODE”设置模式。
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    CS_H;
    delay_us(16);
}
//振动设置
void CPS2::PS2_VibrationMode(void)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  
    PS2_Cmd(0x4D);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x00);
    PS2_Cmd(0X01);
    CS_H;
    delay_us(16);
}
//完成并保存配置
void CPS2::PS2_ExitConfing(void)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  
    PS2_Cmd(0x43);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    CS_H;
    delay_us(16);
}
//手柄配置初始化
void CPS2::PS2_SetInit(void)
{
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_EnterConfing();     //进入配置模式
    PS2_TurnOnAnalogMode(); //“红绿灯”配置模式，配置为模拟模式，并选择是否保存
    //PS2_VibrationMode();  //开启震动模式
    PS2_ExitConfing();      //完成并保存配置
}
