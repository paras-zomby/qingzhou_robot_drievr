#include "timer-cpp.h"

#define TIME TIM6->CNT

static void TIM6_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʱ��ʹ��

    TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

//    //ʹ�ܻ���ʧ��ָ����TIM�ж�  //ʹ��
//    TIM_ITConfig(TIM6, TIM_IT_Update ,ENABLE );
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6�ж�
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
//    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
    TIM_Cmd(TIM6, ENABLE);  //ʹ��TIMx����
}

bool CTim::is_unique = 1;

CTim::CTim(CDebug* const _debug)
    :debug(_debug)
{
    if (is_unique)
    {
        TIM6_Int_Init(60000-1,719);  //CNT ����100 ��Ӧ 1ms
        
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
    if(is_used || ms == 0) debug->ErrorHandle("TIM6", "DelayForTime Error");//return false;//����false
    TIME = 0;
    while(TIME < 100*ms);
    return true;
}

bool CTim::WaitForTime(CNT_STATE state, u8 ms)
{
    if( (is_used && state==CNT_START)   //�Ѿ���ռ���޷���ʼ�¼�ʱ
      ||(!is_used && state==CNT_END) //û�п�ʼֱ�ӽ�����
      ||(!is_used && state==CNT_END_AND_RESTART) //û�п�ʼֱ�����¿�ʼ��
      ||((state==CNT_END_AND_RESTART||state==CNT_END) && ms==0) ) //�趨Ҫ��������û�и���ʱʱ��
        debug->ErrorHandle("TIM6", "WaitForTime Error");//return false; //����false
    
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


void TIM6_IRQHandler(void)   //TIM6�ж�
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
//        PBout(0) = !PBout(0);
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}
