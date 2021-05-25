#ifndef __LED_H
#define __LED_H

#include "sys.h"
#include "delay.h"
#include "debug.h"

class CLed{
    static bool is_unique;
public:
    
    CLed();
    ~CLed();
    void Led_Control(bool state);
};

#endif
