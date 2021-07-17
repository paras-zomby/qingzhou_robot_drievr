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
    CUSART::Data_Sended sdata = {0.0f};
    int Lencoder = 0, Rencoder = 0;
    bool PID_switch = false;
    bool KF_switch = false;
    bool POLL_switch = true;
    
    debug.ShowInfo("debug", "Into While");
    while(1)
    {
        tim.WaitForTime(CTim::CNT_START);
        //�ж�ң�����л�ģʽ��ָ��
        //���STM32���Ӱ������»���SELECT�������£��л�ģʽ
        if(key.KEY_Click() || ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_SELECT))
        {
            debug.mode = (debug.mode>=2)? 1:(debug.mode+1);
        }
        //���CIRCLE���������л�Nano����ģʽ
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_CIRCLE))
        {
            debug.mode = 2;
            debug.LED_Control(LED_STATE::LED_OPEN);
            rdata.Angle = rdata.Speed = 0;
            sdata.Lencoder = sdata.Rencoder = 0;
        }
        //���SQUARE���������л��ֶ�����ģʽ
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_SQUARE))
        {
            debug.mode = 1;
            debug.LED_Control(LED_STATE::LED_CLOSE);
        }
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_TRIANGLE))
            PID_switch = true;
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_CROSS))
            PID_switch = false;
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_PAD_UP))
            KF_switch = true;
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_PAD_DOWN))
            KF_switch = false;
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_L3))
            POLL_switch = true;
        if(ps2.PS2_IfKeyBnClicked(PS2_KEY::PS2B_R3))
            POLL_switch = false;
        if(debug.mode == 1) //�ֶ�����ģʽ
        {
            float Speed = POLL_switch?control.SpeedPretreat(ps2.PS2_AnologData(PS2_POLL::PSS_LY)):0;
            float Angle = POLL_switch?control.AnglePretreat(ps2.PS2_AnologData(PS2_POLL::PSS_RX)):0;
            if(ps2.PS2_IfKeyBnPressed(PS2_KEY::PS2B_R2)) Speed = 55.0f;
            if(ps2.PS2_IfKeyBnPressed(PS2_KEY::PS2B_R1)) Speed = -55.0f;
            if(ps2.PS2_IfKeyBnPressed(PS2_KEY::PS2B_L2)) Speed = 25.0f;
            if(ps2.PS2_IfKeyBnPressed(PS2_KEY::PS2B_L1)) Speed = -25.0f;
            
            Lencoder = encoder.Read_LEncoder();
            Rencoder = encoder.Read_REncoder();
            sdata.Lencoder += Lencoder;
            sdata.Rencoder += Rencoder;
            
            if(KF_switch)   Lencoder = control.kallman_filtering_left(Lencoder),
                            Rencoder = control.kallman_filtering_right(Rencoder);
            control.Kinematic_Analysis(Speed, Angle, Lencoder, Rencoder, PID_switch, false);
            
            const float* p = imu.ReadData();
            for(u8 i = 0; i < 9; ++i)
                sdata.data[i] = p[i];
            tim.WaitForTime(CTim::CNT_WAIT_UNTILL, 25);
            
            Lencoder = encoder.Read_LEncoder();
            Rencoder = encoder.Read_REncoder();
            sdata.Lencoder += Lencoder;
            sdata.Rencoder += Rencoder;
            
            if(KF_switch)   Lencoder = control.kallman_filtering_left(Lencoder),
                            Rencoder = control.kallman_filtering_right(Rencoder);
            control.Kinematic_Analysis(Speed, Angle, Lencoder, Rencoder, PID_switch, true);
            
            ps2.PS2_ReadData();
            
            usart.SendData(CUSART::std_head, sizeof(CUSART::std_head));
            usart.SendData(sdata);
            usart.SendData(CUSART::std_tail, sizeof(CUSART::std_tail));
            sdata.Lencoder = 0;
            sdata.Rencoder = 0;
            
            //=============��1����ʾң��������ֵ=======================//
            debug.OLED_ShowString(00,00,"LY");
            debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PS2_POLL::PSS_LY),5,12);
            debug.OLED_ShowString(80,00,"RX");
            debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PS2_POLL::PSS_RX),4,12);
            
            if(KF_switch) debug.OLED_ShowString(90,40,"KF_ON");
            
            debug.OLED_Refresh_Gram();
            debug.OLED_Clear();
            tim.WaitForTime(CTim::CNT_END, 50);
        }
        else if(debug.mode == 2)
        {
            if(usart.IsDataRefreshed()) rdata = usart.RecvData();
            
            Lencoder = encoder.Read_LEncoder();
            Rencoder = encoder.Read_REncoder();
            sdata.Lencoder += Lencoder;
            sdata.Rencoder += Rencoder;
            
            if(KF_switch)   Lencoder = control.kallman_filtering_left(Lencoder),
                            Rencoder = control.kallman_filtering_right(Rencoder);
            control.Kinematic_Analysis(rdata.Speed, rdata.Angle, Lencoder, Rencoder, PID_switch, false);
            
            const float* p = imu.ReadData();
            for(u8 i = 0; i < 9; ++i)
                sdata.data[i] = p[i];
            
            tim.WaitForTime(CTim::CNT_WAIT_UNTILL, 25);
            
            Lencoder = encoder.Read_LEncoder();
            Rencoder = encoder.Read_REncoder();
            sdata.Lencoder += Lencoder;
            sdata.Rencoder += Rencoder;
            
            if(KF_switch)   Lencoder = control.kallman_filtering_left(Lencoder),
                            Rencoder = control.kallman_filtering_right(Rencoder);
            control.Kinematic_Analysis(rdata.Speed, rdata.Angle, Lencoder, Rencoder, PID_switch, true);
            
            ps2.PS2_ReadData();
            
            usart.SendData(CUSART::std_head, sizeof(CUSART::std_head));
            usart.SendData(sdata);
            usart.SendData(CUSART::std_tail, sizeof(CUSART::std_tail));
            sdata.Lencoder = 0;
            sdata.Rencoder = 0;
            
            debug.OLED_ShowString(0,0,"Nano Control");
            
            if(KF_switch) debug.OLED_ShowString(90,40,"KF_ON");
            
            debug.OLED_Refresh_Gram();
            debug.OLED_Clear();
            
            tim.WaitForTime(CTim::CNT_END, 50);
        }
    }
}

