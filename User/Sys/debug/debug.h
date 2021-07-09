#ifndef __ERROR_H__
#define __ERROR_H__

#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "oled.h"
#include "oledfont.h"
#include "led.h"
#include "key-cpp.h"

//通用的宏定义
#define INSHEREHOLD(left,tar,right) if(tar<(left))tar=(left);else if(tar>(right))tar=(right);
#define ABS(a) ((a)>=0?(a):-(a))

enum class LED_STATE:unsigned char{
    LED_CLOSE = 1,
    LED_OPEN = 0,
    LED_FLASH = 2
};//LED 开关状态的枚举

//包括LED、OLED的底层和通过OLED打印INFO的调试函数
class CDebug
{
    static bool is_unique;
    u8 OLED_GRAM[128][8];
    
    //依赖的对象
    CKey* const key;
    
    //OLED部分
    void OLED_Init(void);
    void OLED_WR_Byte(u8 dat,u8 cmd);
    
    //LED部分
    u8 flashtime;
public:
    u8 mode;
    CDebug(CKey* const _key);
    ~CDebug();
    
    //DEBUG部分
    void ErrorHandle(const char* name, const char* _error);
    void InfoHandle(const char* name, const char* _info);
    void ShowInfo(const char* name, const char* _info);
    void DEBUG_GPIO_Init(void);
    void DEBUG_GPIO_Change(void);
    //LED部分
    void LED_Control(enum LED_STATE state, u8 flashtime_100ms = 10);
    void LED_Flash_Support(void);

    //OLED部分
    void OLED_Display_On(void);
    void OLED_Display_Off(void);
    void OLED_Refresh_Gram(void);
    void OLED_Clear(void);
    void OLED_DrawPoint(u8 x,u8 y,u8 t);
    void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
    void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
    void OLED_ShowString(u8 x,u8 y,const char* p);
};

#endif //__ERROR_H__
