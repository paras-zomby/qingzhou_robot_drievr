#ifndef __USRAT3_H__
#define __USRAT3_H__

#include "sys.h"
#include "debug.h"

enum class USART_VS_CMD:unsigned char{
    HEAD = 4,
    TAIL = 5,
    DATA_8 = 0,
    DATA_16 = 1,
    DATA_32 = 3,
    DATA_64 = 7
};

class CUSART
{
private:
    static bool is_unique;
    
    CDebug* const debug;
public:
    const static u8 std_head[3], std_tail[3];
    struct Data_Sended{
        float data[9];
        int Lencoder, Rencoder;
    };
    struct Data_Recieved{
        float Speed, Angle;
    };
    
    CUSART(CDebug* const _debug);
    ~CUSART();
    
    void SendData(const Data_Sended& data);
    void SendData(const u8* dataptr, u16 datasize = 0);
    void SendData_VS(enum USART_VS_CMD cmd, void* data = NULL);
    
    Data_Recieved RecvData(void);
    bool IsDataRefreshed(void);
};


#endif //__USRAT3_H__
