#include "motor-cpp.h"

#define PWMA1   TIM8->CCR2
#define PWMA2   TIM8->CCR1
#define PWMB1   TIM8->CCR4
#define PWMB2   TIM8->CCR3

#define SPEED_MAX 7200 //0 <= SPEEN_MAX <= 7200

#define SERVO   TIM1->CCR1  //舵机引脚

#define SERVO_INIT 1650
#define K 10.00f

static void Motor_PWM_Init(u16 arr,u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);// 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);  //使能GPIO外设时钟使能

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //TIM_CH1 //TIM_CH4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

    TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能	

    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能		
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4预装载使能	 

    TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器

    TIM_Cmd(TIM8, ENABLE);  //使能TIM
} 

static void Servo_PWM_Init(u16 arr,u16 psc)	
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //使能GPIO外设时钟使能
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // TIM1_CH1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    //!修改重复计数器的值为0!

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
    
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4预装载使能
    TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器
    TIM_Cmd(TIM1, ENABLE);  //使能TIM
    TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能
}

bool CMotor::is_unique = 1;

CMotor::CMotor(CDebug* const _debug)
    :debug(_debug)
{
    if(is_unique)
    {
        Motor_PWM_Init(7199, 0);
        Servo_PWM_Init(9999, 71);
        delay_ms(1);
        ServoCompareSet(0);
        is_unique = 0;
    }
    else
        debug->ErrorHandle("Motor", "class has been redefined");
}

CMotor::~CMotor()
{
    is_unique = 1;
}

//设定电机转速
void CMotor::MotorSpeedSet(short l_speed, short r_speed)
{
    INSHEREHOLD(-SPEED_MAX,l_speed,SPEED_MAX)
    INSHEREHOLD(-SPEED_MAX,r_speed,SPEED_MAX)
    if(l_speed>0) PWMA1=7200,PWMA2=7200-l_speed;
    else          PWMA2=7200,PWMA1=7200+l_speed;

    if(r_speed>0) PWMB1=7200,PWMB2=7200-r_speed;
    else          PWMB2=7200,PWMB1=7200+r_speed;
}

//电机设置为刹车
void CMotor::MotorBrake()
{
    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;
    TIM8->CCR3 = 0;
    TIM8->CCR4 = 0;
}

//设定舵机转向角度
void CMotor::ServoCompareSet(float angle)
{
    float Servo = SERVO_INIT - angle*K;
    INSHEREHOLD(1320,Servo,1980)
    SERVO = Servo;
}
