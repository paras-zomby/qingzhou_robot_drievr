#include "pstwo-cpp.h"

#define DELAY_TIME  delay_us(5);

bool CPS2::is_unique = 1;
const u8 CPS2::Comd[2]={0x01,0x42};	//��ʼ�����������

CPS2::CPS2(CDebug* const _debug)
    :debug(_debug)
{
    if (is_unique)
    {
        PS2_GPIO_Config();
        PS2_SetInit();
        is_unique = 0;
    }
    else
    {
        debug->ErrorHandle("PS2", "class has been redefined");
    }
}

CPS2::~CPS2()
{
    is_unique = 1;
}

void CPS2::PS2_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE); //ʹ�ܶ˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	            //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
    GPIO_Init(GPIOC, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_3;    //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //50M
    GPIO_Init(GPIOC, &GPIO_InitStructure);                  //�����趨������ʼ��GPIOA

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_4;                //�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //50M
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  //�����趨������ʼ��GPIOB
}


//���ֱ���������
void CPS2::PS2_Cmd(u8 CMD)
{
    volatile u16 ref=0x01;
    Data[1] = 0;
    for(ref=0x01;ref<0x0100;ref<<=1)
    {
        if(ref&CMD) DO_H;             //���һλ����λ
        else DO_L;

        CLK_H;                        //ʱ������
        DELAY_TIME;
        CLK_L;
        DELAY_TIME;
        CLK_H;
        if(DI) Data[1] = ref|Data[1];
    }
    delay_us(16);
}
//�ж��Ƿ�Ϊ���ģʽ,0x41=ģ���̵ƣ�0x73=ģ����
//����ֵ��1�����ģʽ
//        ����������ģʽ
bool CPS2::PS2_RedLight(void)
{
    CS_L;
    PS2_Cmd(Comd[0]);  //��ʼ����
    PS2_Cmd(Comd[1]);  //��������
    CS_H;
    if( Data[1] == 0X73)   return 1 ;
    else return 0;
}
//��ȡ�ֱ�����
void CPS2::PS2_ReadData(void)
{
    PS2_ClearData();
    volatile u8 byte=0;
    volatile u16 ref=0x01;
    CS_L;
    PS2_Cmd(Comd[0]);  //��ʼ����
    PS2_Cmd(Comd[1]);  //��������
    for(byte=2;byte<9;++byte)          //��ʼ��������
    {
        for(ref=0x01;ref<0x100;ref<<=1)
        {
            CLK_H;
            DELAY_TIME;
            CLK_L;
            DELAY_TIME;
            CLK_H;
              if(DI)
              Data[byte] = ref|Data[byte];
        }
        delay_us(16);
    }
    CS_H;
    HandKey=~((Data[3]<<8)|Data[4]); //����16���������������洢���޷���16λ����
                                     //���У�ԭ���ݰ��µİ�����Ӧλ��0��û���µİ�����Ӧλ��1��
                                     //ͨ��ȡ����������ɰ���1�ɿ�0��
}

//�԰������д��������������Ҫ���İ��������ԣ���λ�룡
//�������밴��ֻҪ��һ�����¾ͻ᷵����
//ֻ��������أ�����ͬʱ����İ����ֿ������һ�ΰ���
bool CPS2::PS2_IfKeyBnClicked(enum PS2_KEY _key)
{
    u16 key = (u16)_key;
    static u16 pressed_key = 0;
    if((HandKey & key) && !(pressed_key & key))
    {
        pressed_key |= (key & HandKey);
        return true;
    }
    else
    {
        pressed_key &= HandKey;
        return false;
    }
}

bool CPS2::PS2_IfKeyBnPressed(enum PS2_KEY _key)
{
    return (HandKey & (u16)_key);
}

//�õ�һ��ҡ�˵�ģ����  ��Χ0~256
u8 CPS2::PS2_AnologData(enum PS2_POLL _poll)
{
    return Data[(u8)_poll];
}

//������ݻ�����
void CPS2::PS2_ClearData()
{
    u8 a;
    for(a=0;a<9;a++)
        Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	   motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void CPS2::PS2_Vibration(u8 motor1, u8 motor2)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  //��ʼ����
    PS2_Cmd(0x42);  //��������
    PS2_Cmd(0X00);
    PS2_Cmd(motor1);
    PS2_Cmd(motor2);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    CS_H;
    delay_us(16);  
}
//short poll
void CPS2::PS2_ShortPoll(void)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  
    PS2_Cmd(0x42);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x00);
    CS_H;
    delay_us(16);
}
//��������
void CPS2::PS2_EnterConfing(void)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  
    PS2_Cmd(0x43);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x01);
    PS2_Cmd(0x00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    CS_H;
    delay_us(16);
}
//����ģʽ����
void CPS2::PS2_TurnOnAnalogMode(void)
{
    CS_L;
    PS2_Cmd(0x01);  
    PS2_Cmd(0x44);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x01); //analog=0x01;digital=0x00  ������÷���ģʽ
    PS2_Cmd(0x03); //Ox03�������ã�������ͨ��������MODE������ģʽ��
                   //0xEE������������ã���ͨ��������MODE������ģʽ��
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    PS2_Cmd(0X00);
    CS_H;
    delay_us(16);
}
//������
void CPS2::PS2_VibrationMode(void)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  
    PS2_Cmd(0x4D);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x00);
    PS2_Cmd(0X01);
    CS_H;
    delay_us(16);
}
//��ɲ���������
void CPS2::PS2_ExitConfing(void)
{
    CS_L;
    delay_us(16);
    PS2_Cmd(0x01);  
    PS2_Cmd(0x43);  
    PS2_Cmd(0X00);
    PS2_Cmd(0x00);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    PS2_Cmd(0x5A);
    CS_H;
    delay_us(16);
}
//�ֱ����ó�ʼ��
void CPS2::PS2_SetInit(void)
{
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_EnterConfing();     //��������ģʽ
    PS2_TurnOnAnalogMode(); //�����̵ơ�����ģʽ������Ϊģ��ģʽ����ѡ���Ƿ񱣴�
    //PS2_VibrationMode();  //������ģʽ
    PS2_ExitConfing();      //��ɲ���������
}
