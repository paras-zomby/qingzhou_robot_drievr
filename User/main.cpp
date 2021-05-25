#include "debug.h"
#include "key-cpp.h"
#include "pstwo-cpp.h"
#include "motor-cpp.h"
#include "encoder-cpp.h"
#include "timer-cpp.h"
#include "usart-cpp.h"
#include "imu-cpp.h"
#include "control.h"


static void System_init()
{
    RCC->APB2ENR|=1<<0;     //开启AFIO时钟
    AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
    AFIO->MAPR|=1<<25;      //设置JTAG模式为SWD使能，JTAG失能，
                            //防止引脚和编码器的TIM2冲突。
    //最基本的延时函数初始化
    delay_init();
}


int main()
{
    System_init();  //系统基础初始化

    //底层调试类的对象构建
    CKey key;
    CDebug debug(&key);
    //底层硬件控制类的对象构建
    CEncoder encoder(&debug);
    CTim tim(&debug);
    CPS2 ps2(&debug);
    CMotor motor(&debug);
    CUSART usart(&debug);
    
    //上层控制类的对象构建
    CImu imu(&debug);
    CControl control(&debug, &encoder, &motor);
    
    
    debug.InfoHandle("debug", "Into While");
    debug.OLED_Clear();
    while(1)
    {
        tim.WaitForTime(CTim::CNT_START);
        //按键和LED的支持函数
        key.KEY_Long_Press_Support();
        debug.LED_Flash_Support();
        if(key.KEY_Click())
            debug.mode = (debug.mode>=2)? 1:(debug.mode+1);
        //读取遥控器按键
        ps2.PS2_ReadData();
        //执行遥控器命令
        control.Kinematic_Analysis(control.SpeedPretreat(ps2.PS2_AnologData(PSS_LY)),
                                    control.AnglePretreat(ps2.PS2_AnologData(PSS_RX)));
//        //=============第1行显示遥控器接收值=======================//
        debug.OLED_ShowString(00,00,"LY");
        debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PSS_LY),5,12);
        debug.OLED_ShowString(80,00,"RX");
        debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PSS_RX),4,12);

        tim.WaitForTime(CTim::CNT_END, 20);         //总周期20ms
    }
}
//    //=============第1行显示遥控器接收值=======================//
//    debug.OLED_ShowString(00,00,"LY");
//    debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PSS_LY),5,12);
//    debug.OLED_ShowString(80,00,"RX");
//    debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PSS_RX),4,12);

