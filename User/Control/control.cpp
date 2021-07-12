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
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能端口时钟
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;          //端口配置
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
        GPIO_Init(GPIOB, &GPIO_InitStructure);      //根据设定参数初始化GPIO
        
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
    int Bias = Target - Encoder;                //计算偏差
    Pwm += Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
    INSHEREHOLD(-7200, Pwm, 7200)
    Last_bias=Bias;                    //保存上一次偏差
    
    if(ABS(Encoder)<3 && Target==0) Pwm=0,Last_bias=0;
    
    return (short)Pwm;                         //增量输出
}

short CControl::Incremental_PI_Right(int Encoder,int Target)
{
    static int Bias,Pwm,Last_bias;
    Bias = Target - Encoder;                //计算偏差
    Pwm += Velocity_KP*(Bias-Last_bias) + Velocity_KI*Bias;   //增量式PI控制器
    INSHEREHOLD(-7200, Pwm, 7200)
    Last_bias=Bias;                   //保存上一次偏差
    
    if(ABS(Encoder)<3 && Target==0) Pwm=0,Last_bias=0;
    
    return (short)Pwm;                         //增量输出
}

float CControl::kallman_filtering_left(float N_t)
{
    static int N;

    static double P = 0;

    float K,R=0.5,Q=0.10;//R=0.5,Q=0.1

    N=N;//+a*t   //速度获得，由于智能车全程中以匀速运动居多，估计值

    P=P+Q;  // 当前预估计值的协方差矩阵

    K=P/(P+R);// 卡尔曼系数

    N=N+K*(N_t-N);//矫正——得到最优质

    P=(1-K);//最优值的协方差矩阵
    
    return N;
}

float CControl::kallman_filtering_right(float N_t)
{
    static int N;

    static double P = 0;

    float K,R=0.5,Q=0.10;//R=0.5,Q=0.1

    N=N;//+a*t   //速度获得，由于智能车全程中以匀速运动居多，估计值

    P=P+Q;  // 当前预估计值的协方差矩阵

    K=P/(P+R);// 卡尔曼系数

    N=N+K*(N_t-N);//矫正——得到最优质

    P=(1-K);//最优值的协方差矩阵
    
    return N;
}

void CControl::Kinematic_Analysis(float velocity,float angle, int Lencoder, int Rencoder, bool PID_swtich)
{
    INSHEREHOLD(-70, velocity, 70)  //为了保证差速的有效性而进行的限幅,比直线电机速度最大值要小。
    INSHEREHOLD(-23.3f, angle, 23.3f)  //同时限制了输入的电机速度和舵机角度。
    
    int velocity_lf, velocity_rt;
    
    
    double Tand = tan(angle/57.3);                 //Tand = tan(θ)，左转tan(θ)>0;右转tan(θ)<0。
                                                   //C是从舵机转角换算到小车转向角θ的转换参数。
    
    //左右后轮的差速公式
    velocity_lf = velocity*(1+T*Tand/2/L);
    velocity_rt = velocity*(1-T*Tand/2/L);
    
    //=============第2行显示左右电机差速后速度理论值=======================//
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
            //=============第3行显示PWM的实际设定值=======================//
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
    //=============第4行显示编码器的状态=======================//
    if(Lencoder<0) debug->OLED_ShowString(00,30,"-"),
                        debug->OLED_ShowNumber(15,30,-Lencoder,5,12);
    else                    debug->OLED_ShowString(0,30,"+"),
                           debug->OLED_ShowNumber(15,30, Lencoder,5,12);
    if( Rencoder<0)    debug->OLED_ShowString(80,30,"-"),
                           debug->OLED_ShowNumber(95,30,-Rencoder,5,12);
    else                    debug->OLED_ShowString(80,30,"+"),
                           debug->OLED_ShowNumber(95,30, Rencoder,5,12);

    //=============第5行显示舵机的状态=======================//
    debug->OLED_ShowString(00,40, "Servo:"),                //舵机状态
    debug->OLED_ShowNumber(60,40, 1510-10.00*angle,4,12);
    //
        //=============第6行显示速度设定值和打角的设定值=======================//
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
