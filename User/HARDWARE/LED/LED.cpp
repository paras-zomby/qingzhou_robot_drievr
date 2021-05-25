#include "led-cpp.h"

bool CLed::is_unique = 1;
CLed::CLed()
{
    if(is_unique)
    {
        
        is_unique = 0;
    }
    else
        ErrorHandle("LED", "class has been redefined");
}

CLed::~CLed()
{
    is_unique = 0;
}

void CLed::Led_Control(bool state)
{
    PBout(13) = state;
}
