#ifndef __TIMER_CPP_H__
#define __TIMER_CPP_H__

#include "debug.h"

class CTim
{
private:
    static bool is_unique;

    bool is_used;

    CDebug* const debug;

public:
    CTim(CDebug* const _debug);
    ~CTim();
    
    enum CNT_STATE{
        CNT_START = 0,
        CNT_END = 1,
        CNT_END_AND_RESTART = 2
    };
    
    bool DelayForTime(u8 ms);
    bool WaitForTime(CNT_STATE state, u8 ms = 0);
    
};

#endif //__TIMER_CPP_H__
