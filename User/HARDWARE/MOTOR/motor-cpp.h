#ifndef __MOTOR_CPP_H__
#define __MOTOR_CPP_H__

#include "debug.h"



class CMotor
{
private:
    static bool is_unique;
    CDebug* const debug;
    
public:
    CMotor(CDebug* const _debug);
    ~CMotor();
    void MotorSpeedSet(short l_speed, short r_speed);
    void MotorBrake();
    void ServoCompareSet(float angle);
};

#endif //__MOTOR_CPP_H__
