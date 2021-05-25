#ifndef __USRAT3_H__
#define __USRAT3_H__

#include "sys.h"
#include "debug.h"

class CUSART
{
private:
    static bool is_unique;
    
    CDebug* const debug;
public:
    struct Data_Sended{
        
    };
    struct Data_Recieved{
        
    };
    
    CUSART(CDebug* const _debug);
    ~CUSART();
    
    void SendData(const Data_Sended& data);
    void SendData(u8* dataptr, u16 datasize = 0);
    
    Data_Recieved RecvData(void);
    bool IsDataRefreshed(void);
};


#endif //__USRAT3_H__