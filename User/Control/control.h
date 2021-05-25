#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "debug.h"
#include "motor-cpp.h"
#include "pstwo-cpp.h"
#include "encoder-cpp.h"

class CControl
{
private:
    static bool is_unique;

    CDebug* const debug;
    CEncoder* const encoder;
    CMotor* const motor;

    float Velocity_KP,Velocity_KI;
    short Incremental_PI_Left (int Encoder,int Target);
    short Incremental_PI_Right (int Encoder,int Target);
public:
    CControl(CDebug* const _debug, CEncoder* const _encoder, CMotor* const _motor);
    ~CControl();
    
    void Kinematic_Analysis(float velocity,float angle);
    int SpeedPretreat(u8 PSS);
    float AnglePretreat(u8 PSS);

};

#endif //__CONTROL_H__
