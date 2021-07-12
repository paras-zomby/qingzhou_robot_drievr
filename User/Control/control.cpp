#include "control.h"

#define PI 3.141592654
#define T 0.245f
#define L 0.29f


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
    else
        debug->ErrorHandle("Control", "Class has been redefined");
}

CControl::~CControl()
{
    is_unique = 1;
}


short CControl::Incremental_PI_Left(int Encoder,int Target)
{
    static int Pwm,Last_bias;
    int Bias = Target - Encoder;                //����ƫ��
    Pwm += Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
    INSHEREHOLD(-7200, Pwm, 7200)
    Last_bias=Bias;                    //������һ��ƫ��
    
    if(ABS(Encoder)<3 && Target==0) Pwm=0,Last_bias=0;
    
    return (short)Pwm;                         //�������
}

short CControl::Incremental_PI_Right(int Encoder,int Target)
{
    static int Bias,Pwm,Last_bias;
    Bias = Target - Encoder;                //����ƫ��
    Pwm += Velocity_KP*(Bias-Last_bias) + Velocity_KI*Bias;   //����ʽPI������
    INSHEREHOLD(-7200, Pwm, 7200)
    Last_bias=Bias;                   //������һ��ƫ��
    
    if(ABS(Encoder)<3 && Target==0) Pwm=0,Last_bias=0;
    
    return (short)Pwm;                         //�������
}

float CControl::kallman_filtering_left(float N_t)
{
    static int N;

    static double P = 0;

    float K,R=0.5,Q=0.10;//R=0.5,Q=0.1

    N=N;//+a*t   //�ٶȻ�ã��������ܳ�ȫ�����������˶��Ӷ࣬����ֵ

    P=P+Q;  // ��ǰԤ����ֵ��Э�������

    K=P/(P+R);// ������ϵ��

    N=N+K*(N_t-N);//���������õ�������

    P=(1-K);//����ֵ��Э�������
    
    return N;
}

float CControl::kallman_filtering_right(float N_t)
{
    static int N;

    static double P = 0;

    float K,R=0.5,Q=0.10;//R=0.5,Q=0.1

    N=N;//+a*t   //�ٶȻ�ã��������ܳ�ȫ�����������˶��Ӷ࣬����ֵ

    P=P+Q;  // ��ǰԤ����ֵ��Э�������

    K=P/(P+R);// ������ϵ��

    N=N+K*(N_t-N);//���������õ�������

    P=(1-K);//����ֵ��Э�������
    
    return N;
}

void CControl::Kinematic_Analysis(float velocity,float angle, int Lencoder, int Rencoder, bool PID_swtich)
{
    INSHEREHOLD(-70, velocity, 70)  //Ϊ�˱�֤���ٵ���Ч�Զ����е��޷�,��ֱ�ߵ���ٶ����ֵҪС��
    INSHEREHOLD(-23.3f, angle, 23.3f)  //ͬʱ����������ĵ���ٶȺͶ���Ƕȡ�
    
    int velocity_lf, velocity_rt;
    
    
    double Tand = tan(angle/57.3);                 //Tand = tan(��)����תtan(��)>0;��תtan(��)<0��
                                                   //C�ǴӶ��ת�ǻ��㵽С��ת��Ǧȵ�ת��������
    
    //���Һ��ֵĲ��ٹ�ʽ
    velocity_lf = velocity*(1+T*Tand/2/L);
    velocity_rt = velocity*(1-T*Tand/2/L);
    
    //=============��2����ʾ���ҵ�����ٺ��ٶ�����ֵ=======================//
    if( velocity_lf<0)  debug->OLED_ShowString(00,10,"-"),
                       debug->OLED_ShowNumber(15,10,-velocity_lf,5,12);
    else                debug->OLED_ShowString(0,10,"+"),
                       debug->OLED_ShowNumber(15,10, velocity_lf,5,12);
    if( velocity_rt<0) debug->OLED_ShowString(80,10,"-"),
                       debug->OLED_ShowNumber(95,10,-velocity_rt,5,12);
    else                debug->OLED_ShowString(80,10,"+"),
                       debug->OLED_ShowNumber(95,10, velocity_rt,5,12);
    
    if(PID_swtich)
    {
        
        velocity_lf = Incremental_PI_Left(Lencoder, velocity_lf);
        velocity_rt = Incremental_PI_Right(Rencoder, velocity_rt);

        motor->MotorSpeedSet(velocity_lf, velocity_rt);
            //=============��3����ʾPWM��ʵ���趨ֵ=======================//
        if( velocity_lf<0)  debug->OLED_ShowString(00,20,"-"),
                           debug->OLED_ShowNumber(15,20,-velocity_lf,5,12);
        else                debug->OLED_ShowString(0,20,"+"),
                           debug->OLED_ShowNumber(15,20, velocity_lf,5,12);
        if( velocity_rt<0) debug->OLED_ShowString(80,20,"-"),
                           debug->OLED_ShowNumber(95,20,-velocity_rt,5,12);
        else                debug->OLED_ShowString(80,20,"+"),
                           debug->OLED_ShowNumber(95,20, velocity_rt,5,12);
    }
    else
        motor->MotorSpeedSet(velocity_lf*100, velocity_rt*100);
    
    motor->ServoCompareSet(angle);
    
    //*************************************************************
    //=============��4����ʾ��������״̬=======================//
    if(Lencoder<0) debug->OLED_ShowString(00,30,"-"),
                        debug->OLED_ShowNumber(15,30,-Lencoder,5,12);
    else                    debug->OLED_ShowString(0,30,"+"),
                           debug->OLED_ShowNumber(15,30, Lencoder,5,12);
    if( Rencoder<0)    debug->OLED_ShowString(80,30,"-"),
                           debug->OLED_ShowNumber(95,30,-Rencoder,5,12);
    else                    debug->OLED_ShowString(80,30,"+"),
                           debug->OLED_ShowNumber(95,30, Rencoder,5,12);

    //=============��5����ʾ�����״̬=======================//
    debug->OLED_ShowString(00,40, "Servo:"),                //���״̬
    debug->OLED_ShowNumber(60,40, 1510-10.00*angle,4,12);
    //
        //=============��6����ʾ�ٶ��趨ֵ�ʹ�ǵ��趨ֵ=======================//
    if( velocity<0)  debug->OLED_ShowString(00,50,"-"),
                       debug->OLED_ShowNumber(15,50,-velocity,5,12);
    else                debug->OLED_ShowString(0,50,"+"),
                       debug->OLED_ShowNumber(15,50, velocity,5,12);
    if( angle<0) debug->OLED_ShowString(80,50,"-"),
                       debug->OLED_ShowNumber(95,50,-angle,5,12);
    else                debug->OLED_ShowString(80,50,"+"),
                       debug->OLED_ShowNumber(95,50, angle,5,12);
}

float CControl::SpeedPretreat(u8 PSS)
{
    if(ABS(PSS-128)>2)
        return (127-PSS)*2/3.0f;
    else
        return 0.0f;
    
}

float CControl::AnglePretreat(u8 PSS)
{
    if(ABS(PSS-128)>3)
        return (PSS-128)/5.45f;
    else
        return 0.0f;
}
