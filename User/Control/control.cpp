#include "control.h"

#define PI 3.141592654
#define T 0.245f
#define L 0.29f


#define PRINT_DEBUG_INFO(...) {char* pstr=NULL;asprintf(&pstr,__VA_ARGS__);debug->InfoHandle("CONTROL",pstr);free(pstr);}
#define PRINT_ERROR_INFO(...) {char* pstr=NULL;asprintf(&pstr,__VA_ARGS__);debug->ErrorHandle("CONTROL",pstr);free(pstr);}


bool CControl::is_unique = 1;

CControl::CControl(CDebug* const _debug,CEncoder* const _encoder, CMotor* const _motor)
    :debug(_debug)
    ,encoder(_encoder)
    ,motor(_motor)
    ,Velocity_KP(62)
    ,Velocity_KI(62)
{
    if (is_unique)
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ�ܶ˿�ʱ��
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;          //�˿�����
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
        GPIO_Init(GPIOB, &GPIO_InitStructure);      //�����趨������ʼ��GPIO
        
        is_unique = 0;
    }
}

CControl::~CControl()
{
    is_unique = 1;
}


short CControl::Incremental_PI_Left(int Encoder,int Target)
{
    static int Pwm,Last_bias;
    int Bias = Encoder-Target;                //����ƫ��
    Pwm += Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
    INSHEREHOLD(-7200, Pwm, 7200)
    Last_bias=Bias;	                   //������һ��ƫ��
    
    if(ABS(Encoder)<3&&Target==0) Pwm=0,Last_bias=0;
    
    return (short)Pwm;                         //�������
}

short CControl::Incremental_PI_Right(int Encoder,int Target)
{
    static int Bias,Pwm,Last_bias;
    Bias = Encoder-Target;                //����ƫ��
    Pwm += Velocity_KP*(Bias-Last_bias) + Velocity_KI*Bias;   //����ʽPI������
    INSHEREHOLD(-7200, Pwm, 7200)
    Last_bias=Bias;                   //������һ��ƫ��
    if(ABS(Encoder)<3&&Target==0) Pwm=0,Last_bias=0;
    
    return (short)Pwm;                         //�������
}

void CControl::Kinematic_Analysis(float velocity,float angle)
{
    int Lencoder,Rencoder;
    int velocity_lf,velocity_rt;
    
    
    double Tand = tan(angle/57.3);                 //Tand = tan(��)����תtan(��)>0;��תtan(��)<0��
                                                   //C�ǴӶ��ת�ǻ��㵽С��ת��Ǧȵ�ת��������
    
    Lencoder = encoder->Read_LEncoder();
    Rencoder = encoder->Read_REncoder();
    
    
    //���Һ��ֵĲ��ٹ�ʽ
    velocity_lf = velocity*(1-T*Tand/2/L);
    velocity_rt = -velocity*(1+T*Tand/2/L);
    
    
    //=============��2����ʾ���ҵ���ٶ�����ֵ=======================//
    if( velocity_lf<0)  debug->OLED_ShowString(00,10,"-"),
                       debug->OLED_ShowNumber(15,10,-velocity_lf,5,12);
    else                debug->OLED_ShowString(0,10,"+"),
                       debug->OLED_ShowNumber(15,10, velocity_lf,5,12);
    if( velocity_rt<0) debug->OLED_ShowString(80,10,"-"),
                       debug->OLED_ShowNumber(95,10,-velocity_rt,4,12);
    else                debug->OLED_ShowString(80,10,"+"),
                       debug->OLED_ShowNumber(95,10, velocity_rt,4,12);

    velocity_lf = Incremental_PI_Left(Lencoder, velocity_lf);
    velocity_rt = Incremental_PI_Right(Rencoder, velocity_rt);

    motor->MotorSpeedSet(velocity_lf, velocity_rt);
    motor->ServoCompareSet(angle);
    
    //*************************************************************
    //=============��3����ʾPWM��ʵ���趨ֵ=======================//
    if( velocity_lf<0)  debug->OLED_ShowString(00,20,"-"),
                       debug->OLED_ShowNumber(15,20,-velocity_lf,5,12);
    else                debug->OLED_ShowString(0,20,"+"),
                       debug->OLED_ShowNumber(15,20, velocity_lf,5,12);
    if( velocity_rt<0) debug->OLED_ShowString(80,20,"-"),
                       debug->OLED_ShowNumber(95,20,-velocity_rt,4,12);
    else                debug->OLED_ShowString(80,20,"+"),
                       debug->OLED_ShowNumber(95,20, velocity_rt,4,12);
    //=============��4����ʾ��������״̬=======================//
    if(Lencoder<0) debug->OLED_ShowString(00,30,"-"),
                        debug->OLED_ShowNumber(15,30,-Lencoder,5,12);
    else                    debug->OLED_ShowString(0,30,"+"),
                           debug->OLED_ShowNumber(15,30, Lencoder,5,12);
    if( Rencoder<0)    debug->OLED_ShowString(80,30,"-"),
                           debug->OLED_ShowNumber(95,30,-Rencoder,4,12);
    else                    debug->OLED_ShowString(80,30,"+"),
                           debug->OLED_ShowNumber(95,30, Rencoder,4,12);

    //=============��5����ʾ�����״̬=======================//
    debug->OLED_ShowString(00,40, "Servo:"),                //���״̬
    debug->OLED_ShowNumber(60,40, 1650-10.00*angle,4,12);
    //
        //=============��6����ʾ�ٶ��趨ֵ�ʹ�ǵ��趨ֵ=======================//
    if( velocity<0)  debug->OLED_ShowString(00,50,"-"),
                       debug->OLED_ShowNumber(15,50,-velocity,5,12);
    else                debug->OLED_ShowString(0,50,"+"),
                       debug->OLED_ShowNumber(15,50, velocity,5,12);
    if( angle<0) debug->OLED_ShowString(80,50,"-"),
                       debug->OLED_ShowNumber(95,50,-angle,4,12);
    else                debug->OLED_ShowString(80,50,"+"),
                       debug->OLED_ShowNumber(95,50, angle,4,12);
    debug->OLED_Refresh_Gram();
}

int CControl::SpeedPretreat(u8 PSS)
{
    return (PSS-127)*2/3;
}

float CControl::AnglePretreat(u8 PSS)
{
    if(ABS(PSS-128)>3)
        return (PSS-128)/3.6f;
    else
        return 0.0f;
}
