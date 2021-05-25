#ifndef __KEY_CPP_H__
#define __KEY_CPP_H__

#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"

#define KEY PBin(14)

class CKey
{
private:
    static bool is_unique;

    u16 pressed_time;
public:
    CKey();
    ~CKey();
    
    void KEY_Long_Press_Support(void);
    
    bool KEY_Click();
    bool KEY_Long_Press(void);
    
};

#endif //__KEY_CPP_H__
