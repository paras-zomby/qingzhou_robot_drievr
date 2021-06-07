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
    RCC->APB2ENR|=1<<0;     //����AFIOʱ��
    AFIO->MAPR&=0XF8FFFFFF; //���MAPR��[26:24]
    AFIO->MAPR|=1<<25;      //����JTAGģʽΪSWDʹ�ܣ�JTAGʧ�ܣ�
                            //��ֹ���źͱ�������TIM2��ͻ��
    //���������ʱ������ʼ��
    delay_init();
}


int main()
{
    System_init();  //ϵͳ������ʼ��

    //�ײ������Ķ��󹹽�
    CKey key;
    CDebug debug(&key);
    //�ײ�Ӳ��������Ķ��󹹽�
    CEncoder encoder(&debug);
    CTim tim(&debug);
    CPS2 ps2(&debug);
    CMotor motor(&debug);
    CUSART usart(&debug);
    
    //�ϲ������Ķ��󹹽�
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
        //������LED��֧�ֺ���
        key.KEY_Long_Press_Support();
        debug.LED_Flash_Support();
        if(key.KEY_Click())
            debug.mode = (debug.mode>=2)? 1:(debug.mode+1);
        if(debug.mode == 1)
        {
            //��ȡң��������
            ps2.PS2_ReadData();
//            //���select���������л�ģʽ
//            if(PSB_SELECT & ps2.PS2_ReturnPressedKey())
//                debug.mode = 2;
            //ִ��ң��������
            control.Kinematic_Analysis(control.SpeedPretreat(ps2.PS2_AnologData(PSS_LY)),
                                        control.AnglePretreat(ps2.PS2_AnologData(PSS_RX)));
            //=============��1����ʾң��������ֵ=======================//
            debug.OLED_ShowString(00,00,"LY");
            debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PSS_LY),5,12);
            debug.OLED_ShowString(80,00,"RX");
            debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PSS_RX),4,12);
            
            debug.OLED_Refresh_Gram();
            tim.WaitForTime(CTim::CNT_END, 20);         //������20ms
        }
        else if(debug.mode == 2)
        {
            if(usart.IsDataRefreshed()) rdata = usart.RecvData();
            
//            if(time_flag == 9)// 100ms per time: 9
//            {
//                ps2.PS2_ReadData();
//                //���select���������л�ģʽ
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
            tim.WaitForTime(CTim::CNT_END, 10);         //��������10ms
        }
        
        
    }
}

