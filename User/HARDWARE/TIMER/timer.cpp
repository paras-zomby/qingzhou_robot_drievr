#include "timer-cpp.h"

#define TIME TIM6->CNT

static void TIM6_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能

    TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

//    //使能或者失能指定的TIM中断  //使能
//    TIM_ITConfig(TIM6, TIM_IT_Update ,ENABLE );
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6中断
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
    TIM_Cmd(TIM6, ENABLE);  //使能TIMx外设
}

bool CTim::is_unique = 1;

CTim::CTim(CDebug* const _debug)
    :debug(_debug)
{
    if (is_unique)
    {
        TIM6_Int_Init(60000-1,719);  //CNT 自增100 对应 1ms
        
        is_used = false;
        is_unique = 0;
    }
    else
        debug->ErrorHandle("timer", "class has been redefined");
}

CTim::~CTim()
{
    is_unique = 1;
}

bool CTim::DelayForTime(u8 ms)
{
    if(is_used || ms == 0) debug->ErrorHandle("TIM6", "DelayForTime Error");//return false;//返回false
    TIME = 0;
    while(TIME < 100*ms);
    return true;
}

bool CTim::WaitForTime(CNT_STATE state, u8 ms)
{
    if( (is_used && state==CNT_START)   //已经被占用无法开始新计时
      ||(!is_used && state==CNT_END) //没有开始直接结束了
      ||(!is_used && state==CNT_END_AND_RESTART) //没有开始直接重新开始了
      ||((state==CNT_END_AND_RESTART||state==CNT_END) && ms==0) ) //设定要结束但是没有给延时时间
        debug->ErrorHandle("TIM6", "WaitForTime Error");//return false; //返回false
    
    if(state==CNT_START)
    {
        TIME = 0;
        is_used = true;
    }
    else if(state==CNT_END_AND_RESTART)
    {
        while(TIME < 100*ms);
        TIME = 0;
    }
    else
    {
        while(TIME < 100*ms);
        is_used = false;
    }
    return true;
}


void TIM6_IRQHandler(void)   //TIM6中断
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
//        PBout(0) = !PBout(0);
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}
