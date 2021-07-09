#ifndef __ERROR_H__
#define __ERROR_H__

#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "oled.h"
#include "oledfont.h"
#include "led.h"
#include "key-cpp.h"

//ͨ�õĺ궨��
#define INSHEREHOLD(left,tar,right) if(tar<(left))tar=(left);else if(tar>(right))tar=(right);
#define ABS(a) ((a)>=0?(a):-(a))

enum class LED_STATE:unsigned char{
    LED_CLOSE = 1,
    LED_OPEN = 0,
    LED_FLASH = 2
};//LED ����״̬��ö��

//����LED��OLED�ĵײ��ͨ��OLED��ӡINFO�ĵ��Ժ���
class CDebug
{
    static bool is_unique;
    u8 OLED_GRAM[128][8];
    
    //�����Ķ���
    CKey* const key;
    
    //OLED����
    void OLED_Init(void);
    void OLED_WR_Byte(u8 dat,u8 cmd);
    
    //LED����
    u8 flashtime;
public:
    u8 mode;
    CDebug(CKey* const _key);
    ~CDebug();
    
    //DEBUG����
    void ErrorHandle(const char* name, const char* _error);
    void InfoHandle(const char* name, const char* _info);
    void ShowInfo(const char* name, const char* _info);
    void DEBUG_GPIO_Init(void);
    void DEBUG_GPIO_Change(void);
    //LED����
    void LED_Control(enum LED_STATE state, u8 flashtime_100ms = 10);
    void LED_Flash_Support(void);

    //OLED����
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
