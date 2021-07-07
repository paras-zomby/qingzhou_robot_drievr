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
    
    //临时变量初始化
    CUSART::Data_Recieved rdata = {0.0f, 0.0f};
    CUSART::Data_Sended sdata = {0};
    u8 time_flag = 1;

    debug.ShowInfo("debug", "Into While");
    while(1)
    {
        tim.WaitForTime(CTim::CNT_START);
        //按键和LED的支持函数
        key.KEY_Long_Press_Support();
        debug.LED_Flash_Support();
        if(key.KEY_Click())
        {
            debug.mode = (debug.mode>=2)? 1:(debug.mode+1);
            time_flag = 1;
            debug.OLED_Clear();
            debug.OLED_Refresh_Gram();
        }
            if(debug.mode == 1)
        {
            //读取遥控器按键
            ps2.PS2_ReadData();
            //如果select按键按下切换模式
//            if(ps2.PS2_IfKeyBnClicked())
                debug.mode = 2;
            //执行遥控器命令
            int Lencoder = encoder.Read_LEncoder();
            int Rencoder = encoder.Read_REncoder();
            control.Kinematic_Analysis(control.SpeedPretreat(ps2.PS2_AnologData(PS2_POLL::PSS_LY)),
                                        control.AnglePretreat(ps2.PS2_AnologData(PS2_POLL::PSS_RX)),
                                         Lencoder, Rencoder);
            //=============第1行显示遥控器接收值=======================//
            debug.OLED_ShowString(00,00,"LY");
            debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PS2_POLL::PSS_LY),5,12);
            debug.OLED_ShowString(80,00,"RX");
            debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PS2_POLL::PSS_RX),4,12);
            
            debug.OLED_Refresh_Gram();
            debug.OLED_Clear();
            
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
            {
                int Lencoder = encoder.Read_LEncoder();
                int Rencoder = encoder.Read_REncoder();
                control.Kinematic_Analysis(rdata.Speed, rdata.Angle, Lencoder, Rencoder);
                sdata.Lencoder += Lencoder;
                sdata.Rencoder += Rencoder;
            }
            if(time_flag == 3)// 100ms per time: 3
            {
                //debug.ShowInfo("Nano Control", "");imu.ShowData_OLED();
//                //第三行显示收到的Speed和Angle数据。如果开启上面的control类的控制函数就不用了。
//                debug.OLED_ShowString(0,10,"Nano Control");
//                if(rdata.Angle<0) debug.OLED_ShowString(00,30,"-"),
//                     debug.OLED_ShowNumber(15,30,-(int)rdata.Angle,5,12);
//                else debug.OLED_ShowString(0,30,"+"),
//                     debug.OLED_ShowNumber(15,30, (int)rdata.Angle,5,12);
//                if(rdata.Speed<0) debug.OLED_ShowString(80,30,"-"),
//                      debug.OLED_ShowNumber(95,30,-(int)rdata.Speed,4,12);
//                else  debug.OLED_ShowString(80,30,"+"),
//                      debug.OLED_ShowNumber(95,30, (int)rdata.Speed,4,12);
                
                debug.OLED_Refresh_Gram();
                debug.OLED_Clear();
            }
            
             if(time_flag == 1 || time_flag == 5)// ~~50ms per time: 1,5
            {
                const float* p = imu.ReadData();
                for(u8 i = 0; i < 9; ++i)
                    sdata.data[i] = p[i];
            }
            
            if(time_flag %5 == 2)//50ms per time: 2,7
            {
//                debug.LED_Control(CDebug::LED_OPEN);
                usart.SendData(CUSART::std_head, sizeof(CUSART::std_head));
                usart.SendData(sdata);
                usart.SendData(CUSART::std_tail, sizeof(CUSART::std_tail));
//                debug.LED_Control(CDebug::LED_CLOSE);
            }  
            
            time_flag = time_flag>=10?1:time_flag+1;
            tim.WaitForTime(CTim::CNT_END, 10);         //单次周期10ms
        }
    }
}

