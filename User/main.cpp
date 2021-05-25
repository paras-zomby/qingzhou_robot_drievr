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
    
    
    debug.InfoHandle("debug", "Into While");
    debug.OLED_Clear();
    while(1)
    {
        tim.WaitForTime(CTim::CNT_START);
        //������LED��֧�ֺ���
        key.KEY_Long_Press_Support();
        debug.LED_Flash_Support();
        if(key.KEY_Click())
            debug.mode = (debug.mode>=2)? 1:(debug.mode+1);
        //��ȡң��������
        ps2.PS2_ReadData();
        //ִ��ң��������
        control.Kinematic_Analysis(control.SpeedPretreat(ps2.PS2_AnologData(PSS_LY)),
                                    control.AnglePretreat(ps2.PS2_AnologData(PSS_RX)));
//        //=============��1����ʾң��������ֵ=======================//
        debug.OLED_ShowString(00,00,"LY");
        debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PSS_LY),5,12);
        debug.OLED_ShowString(80,00,"RX");
        debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PSS_RX),4,12);

        tim.WaitForTime(CTim::CNT_END, 20);         //������20ms
    }
}
//    //=============��1����ʾң��������ֵ=======================//
//    debug.OLED_ShowString(00,00,"LY");
//    debug.OLED_ShowNumber(15,00,ps2.PS2_AnologData(PSS_LY),5,12);
//    debug.OLED_ShowString(80,00,"RX");
//    debug.OLED_ShowNumber(95,00,ps2.PS2_AnologData(PSS_RX),4,12);

