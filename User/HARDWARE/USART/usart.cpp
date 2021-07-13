#include "usart-cpp.h"

bool CUSART::is_unique = 1;

const u8 CUSART::std_head[] = {0xb5, 0x02, 0xfb};
const u8 CUSART::std_tail[] = {0xc4, 0x03, 0x0d};

static CUSART::Data_Recieved recvdata;
static bool is_datarefreshed = 0;   //数据是否更新的标志位，
                                    //接收数据的中断函数会把他置1
                                    //调用读数据函数会把他置0

CUSART::CUSART(CDebug* const _debug)
    :debug(_debug)
{
    if(is_unique)
    {
//		GPIO_InitTypeDef GPIO_InitStructure;
//		USART_InitTypeDef USART_InitStructure;

//		// 打开串口GPIO、AFIO的时钟
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);

//		// 将USART Tx的GPIO配置为推挽复用模式
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//		GPIO_Init(GPIOA, &GPIO_InitStructure);

//		// 将USART Rx的GPIO配置为浮空输入模式
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//		GPIO_Init(GPIOA, &GPIO_InitStructure);
//			
			
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
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
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

void CUSART::SendData(const u8* dataptr, u16 datasize)
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

void CUSART::SendData_VS(enum USART_VS_CMD cmd, void* data)
{
    if(cmd == USART_VS_CMD::HEAD)
    {
        USART_SendData(USART1, 0x03);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
        USART_SendData(USART1, 0xfc);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    }
    else if (cmd == USART_VS_CMD::TAIL)
    {
        USART_SendData(USART1, 0xfc);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
        USART_SendData(USART1, 0x03);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
    }
    else if(data != NULL)
    {
        for(u8* ptr = (u8*)data; ptr <= (u8*)data + (u8)cmd; ++ptr)
        {
            USART_SendData(USART1, *ptr);
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
        }
    }
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

extern "C"{

void USART1_IRQHandler(void)
{
    static u8 index = 1;               //表示ptr正在指向recvdata的第几个字节
    static u8* ptr = (u8*)&recvdata;    //指向recvdata的下一个待写入的字节
    
    static u8 is_recving = 0, cnt_head = 0, cnt_tail = 0;//用于标准头、标准尾协议判断。
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET) //如果真的进中断了
    {
        u8 data = (USART1->DR&0xFF);
        if(!is_recving)
        {
            if(data == CUSART::std_head[0]) cnt_head = 1;         //只要接收到的是标准头开头，就把cnt置一（标准头第二个）
            else if(cnt_head != 0 && data == CUSART::std_head[cnt_head])//如果cnt不是0且数据等于std[cnt]
            {
                if (cnt_head < 2) ++cnt_head;         //如果cnt没到std标准头的最后一位，则++cnt
                else                        //cnt已经走到标准头最后一位，接收信息开始
                {
                    is_recving = 1;
                    ptr = (u8*)&recvdata;
                    index = 1;
                }
            }
            else cnt_head = 0;                       //如果上面两种判断都不是，那么cnt回归0
        }
        else
        {
            if(index <= sizeof(recvdata))          //当ptr还指向recvdata的字节，
            {                                      //且正在接收消息
                *ptr = data;
                ++ptr;
            }
             ++index;                               //只要接收到消息就++index
            
            if(data == CUSART::std_tail[0]) cnt_tail = 1;             //只要数据符合数据尾的第一位，就把cnt置1
            else if(cnt_tail != 0 && data == CUSART::std_tail[cnt_tail])   //如果cnt不是2且数据等于std[cnt]
            {
                if (cnt_tail < 2) ++cnt_tail;             //如果cnt没有到标准尾的最后一位，--cnt
                else                            //cnt已经走完标准尾，接收信息结束
                {
                    is_recving = 0;
                    if(index-1 == sizeof(recvdata)+3)
                        is_datarefreshed = 1;
                }
            }
            else cnt_tail = 0;                           //其余结果把cnt恢复到2
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

}
