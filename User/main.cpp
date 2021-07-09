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
    
    //��ʱ������ʼ��
    CUSART::Data_Recieved rdata = {0.0f, 0.0f};
    CUSART::Data_Sended sdata = {0};
    u8 time_flag = 1;

    debug.ShowInfo("debug", "Into While");
    while(1)
    {
        tim.WaitForTime(CTim::CNT_START);
        //������LED��֧�ֺ���
        key.KEY_Long_Press_Support();
        debug.LED_Flash_Support();
        //�ж�ң�����л�ģʽ��ָ��
        //���STM32���Ӱ������»���SELECT�������£��л�ģʽ
        if(key.KEY_Click() || ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_SELECT))
        {
            debug.mode = (debug.mode>=2)? 1:(debug.mode+1);
            time_flag = 1;
        }
        //���CIRCLE���������л�Nano����ģʽ
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_CIRCLE))
        {
            debug.mode = 2;time_flag = 1;
            debug.LED_Control(LED_STATE::LED_OPEN);
        }
        //���SQUARE���������л��ֶ�����ģʽ
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_SQUARE))
        {
            debug.mode = 1;
            debug.LED_Control(LED_STATE::LED_CLOSE);
        }
        if(debug.mode == 1) //�ֶ�����ģʽ
        {
            //��ȡң��������
            ps2.PS2_ReadData();
            //ִ��ң��������
            float Speed = control.SpeedPretreat(ps2.PS2_AnologData(PS2_POLL::PSS_LY));
            float Angle = control.AnglePretreat(ps2.PS2_AnologData(PS2_POLL::PSS_RX));
            
            if(ps2.PS2_IfKeyBnPressed(PS2_KEY::PS2B_L2)) Speed = 55.0f;
            if(ps2.PS2_IfKeyBnPressed(PS2_KEY::PS2B_R2)) Speed = -55.0f;
            
            if(ps2.PS2_IfKeyBnPressed(PS2_KEY::PS2B_PAD_DOWN))
                control.Kinematic_Analysis(0, Angle, encoder.Read_LEncoder(), encoder.Read_REncoder());
            else
                control.Kinematic_Analysis(Speed, Angle, encoder.Read_LEncoder(), encoder.Read_REncoder());
            
            //=============��1����ʾң��������ֵ=======================//
            debug.OLED_ShowString(00,00,"LY");
            debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PS2_POLL::PSS_LY),5,12);
            debug.OLED_ShowString(80,00,"RX");
            debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PS2_POLL::PSS_RX),4,12);
            
            debug.OLED_Refresh_Gram();
            debug.OLED_Clear();
            
            tim.WaitForTime(CTim::CNT_END, 20);         //������20ms
        }
        else if(debug.mode == 2)
        {
            if(usart.IsDataRefreshed()) rdata = usart.RecvData();
            
            if(time_flag == 9)// 100ms per time: 9
            {
                ps2.PS2_ReadData();
            }
            
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
                debug.OLED_ShowString(0,0,"Nano Control");
                //��������ʾ�յ���Speed��Angle���ݡ�������������control��Ŀ��ƺ����Ͳ����ˡ�
//                if(rdata.Angle<0) debug.OLED_ShowString(00,30,"-"),
//                     debug.OLED_ShowNumber(15,30,-(int)rdata.Speed,5,12);
//                else debug.OLED_ShowString(0,30,"+"),
//                     debug.OLED_ShowNumber(15,30, (int)rdata.Speed,5,12);
//                if(rdata.Speed<0) debug.OLED_ShowString(80,30,"-"),
//                      debug.OLED_ShowNumber(95,30,-(int)rdata.Angle,4,12);
//                else  debug.OLED_ShowString(80,30,"+"),
//                      debug.OLED_ShowNumber(95,30, (int)rdata.Angle,4,12);
                
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
                sdata.Lencoder = 0;
                sdata.Rencoder = 0;
//                debug.LED_Control(CDebug::LED_CLOSE);
            }  
            
            time_flag = time_flag>=10?1:time_flag+1;
            tim.WaitForTime(CTim::CNT_END, 10);         //��������10ms
        }
    }
}

