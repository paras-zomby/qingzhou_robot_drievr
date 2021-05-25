#include "usart-cpp.h"

bool CUSART::is_unique = 1;

static CUSART::Data_Recieved recvdata;

static bool is_datarefreshed = 0;   //数据是否更新的标志位，
                                    //接收数据的中断函数会把他置1
                                    //调用读数据函数会把他置0

CUSART::CUSART(CDebug* const _debug)
    :debug(_debug)
{
    if(is_unique)
    {
        //======================USART配置部分==============================
        //GPIO端口设置
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE); //使能USART1，GPIOA时钟

        //USART1_TX   GPIOA.9
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //复用推挽输出
        GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

        //USART1_RX     GPIOA.10初始化
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
        GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  
        //USART 初始化设置
        //UsartNVIC 配置
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      //子优先级
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         //IRQ通道使能
        NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器

        USART_InitStructure.USART_BaudRate = 115200;//串口波特率
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
        USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
        USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式

        USART_Init(USART1, &USART_InitStructure); //初始化串口1
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
        USART_Cmd(USART1, ENABLE);                    //使能串口1
        //==================================================================
        
        is_unique = 0;
    }
    else
        debug->ErrorHandle("usart", "class has been redefined");
}

CUSART::~CUSART()
{
    is_unique = 1;
}

void CUSART::SendData(u8* dataptr, u16 datasize)
{
    while(datasize--)
    {
        USART_SendData(USART1, *dataptr++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    }
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
}

void CUSART::SendData(const Data_Sended& data)
{
    u8* dataptr = (u8*)&data;
    u16 datasize = sizeof(Data_Sended);
    while(datasize--)
    {
        USART_SendData(USART1, *dataptr++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    }
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
}

CUSART::Data_Recieved CUSART::RecvData(void)
{
    is_datarefreshed = 0;
    return recvdata;
}

bool CUSART::IsDataRefreshed(void)
{
    return is_datarefreshed;
}

void USART1_IRQHandler(void)
{
    static u16 index = 1;               //表示ptr正在指向recvdata的第几个字节
    static u8* ptr = (u8*)&recvdata;    //指向recvdata的下一个待写入的字节
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        u8 data = (USART1->DR&0xFF);
        if(index < sizeof(recvdata))    //当ptr还没有指向recvdata的最后一个字节时
        {
            *ptr = data;
            ++ptr; ++index;
        }
        else
        {
            *ptr = data;
            ptr = (u8*)&recvdata;
            index = 1;
            is_datarefreshed = 1;
        }
    }
}
