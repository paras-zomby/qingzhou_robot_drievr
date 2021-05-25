#include "key-cpp.h"

bool CKey::is_unique = 1;

CKey::CKey()
{
    if (is_unique)
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能端口时钟
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; //端口配置
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
        GPIO_Init(GPIOB, &GPIO_InitStructure);      //根据设定参数初始化GPIO
        
        is_unique = 0;
    }
}

CKey::~CKey()
{
    is_unique = 1;
}


/*
********************KEY部分**********************
*/

bool CKey::KEY_Click()
{
    static bool flag_key = 0; //按键标志
    if(PBin(14)==0)
        return flag_key?0:flag_key = 1;
    else
        return flag_key = 0;
}

void CKey::KEY_Long_Press_Support(void)
{
    if(PBin(14)==0)
        ++pressed_time;
    else
        pressed_time = 0;
}

bool CKey::KEY_Long_Press(void)
{
    if(pressed_time > 40)
        return 1;
    else
        return 0;
}

//u8 CKey::_KEY_Click_N_Double (u8 time)
//{
//    static u8 flag_key,count_key,double_key;
//    static u16 count_single,Forever_count;
//    if(KEY==0)  Forever_count++;   //长按标志位未置1
//    else        Forever_count=0;
//    if(0==KEY&&0==flag_key) flag_key=1;
//    if(0==count_key)
//    {
//        if(flag_key==1) 
//        {
//            double_key++;
//            count_key=1;
//        }
//        if(double_key==2) 
//        {
//            double_key=0;
//            count_single=0;
//            return 2;//双击执行的指令
//        }
//    }
//    if(1==KEY)  flag_key=0,count_key=0;

//    if(1==double_key)
//    {
//        count_single++;
//        if(count_single>time&&Forever_count<time)
//        {
//            double_key=0;
//            count_single=0;	
//            return 1;//单击执行的指令
//        }
//        if(Forever_count>time)
//        {
//            double_key=0;
//            count_single=0;
//        }
//    }
//    return 0;
//}
