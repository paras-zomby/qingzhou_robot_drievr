#ifndef __ENCODER_CPP_H__
#define __ENCODER_CPP_H__

#include "stm32f10x.h"
#include "sys.h"
#include "debug.h"

#define ENCODER_TIM_PERIOD (u16)(65535)   //���ɴ���65535 ��ΪF103�Ķ�ʱ����16λ�ġ�

class CEncoder
{
private:
    static bool is_unique;
    CDebug* const debug;
public:
    CEncoder(CDebug* const _debug);
    ~CEncoder();
    int Read_LEncoder();
    int Read_REncoder();
};

#endif //__ENCODER_CPP_H__
