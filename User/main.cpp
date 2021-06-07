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
    
    CUSART::Data_Recieved rdata = {0.0f, 0.0f};
    CUSART::Data_Sended sdata = {0};
    u8 time_flag = 1;
    
    debug.ShowInfo("debug", "Into While");
    while(1)
    {
        tim.WaitForTime(CTim::CNT_START);
        debug.OLED_Clear();
        //按键和LED的支持函数
        key.KEY_Long_Press_Support();
        debug.LED_Flash_Support();
        if(key.KEY_Click())
            debug.mode = (debug.mode>=2)? 1:(debug.mode+1);
        if(debug.mode == 1)
        {
            //读取遥控器按键
            ps2.PS2_ReadData();
//            //如果select按键按下切换模式
//            if(PSB_SELECT & ps2.PS2_ReturnPressedKey())
//                debug.mode = 2;
            //执行遥控器命令
            control.Kinematic_Analysis(control.SpeedPretreat(ps2.PS2_AnologData(PSS_LY)),
                                        control.AnglePretreat(ps2.PS2_AnologData(PSS_RX)));
            //=============第1行显示遥控器接收值=======================//
            debug.OLED_ShowString(00,00,"LY");
            debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PSS_LY),5,12);
            debug.OLED_ShowString(80,00,"RX");
            debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PSS_RX),4,12);
            
            debug.OLED_Refresh_Gram();
            tim.WaitForTime(CTim::CNT_END, 20);         //总周期20ms
        }
        else if(debug.mode == 2)
        {
            if(usart.IsDataRefreshed()) rdata = usart.RecvData();
            
//            if(time_flag == 9)// 100ms per time: 9
//            {
//                ps2.PS2_ReadData();
//                //如果select按键按下切换模式
//                if(PSB_SELECT & ps2.PS2_ReturnPressedKey())
//                    debug.mode = 1;
//            }
            
            if(time_flag%2 == 0)//20ms per time: 2,4,6,8,10
                control.Kinematic_Analysis(rdata.Speed, rdata.Angle);
            
            if(time_flag == 3)// 100ms per time: 3
                {debug.OLED_ShowString(00,00,"Nano Control");imu.ShowData_OLED();}
            
            if(time_flag == 1 || time_flag == 5)// ~~50ms per time: 1,5
            {
                const float* p = imu.ReadData();
                for(u8 i = 0; i < 9; ++i)
                    sdata.data[i] = p[i];
            }
            
            if(time_flag %5 == 2)//50ms per time: 2,7
            {
                usart.SendData(CUSART::std, sizeof(CUSART::std));
                usart.SendData(sdata);
                usart.SendData(CUSART::std, sizeof(CUSART::reverse_std));
            }
            
            time_flag = time_flag>=10?1:time_flag+1;
            tim.WaitForTime(CTim::CNT_END, 10);         //单次周期10ms
        }
        
        
    }
}

