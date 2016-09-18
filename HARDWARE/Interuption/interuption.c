#include "sys.h"
#include "interuption.h"
#include "delay.h"
#include "math.h"
#include "led.h"



//////////////////////////////////////////////////////////////////////////////////	 
//功能：中断初始化
//参数：无
//返回值：无   							  
////////////////////////////////////////////////////////////////////////////////// 
void Int_Init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM4时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1; 	//自动重装载值 初始设置 2ms 中断一次
	TIM_TimeBaseInitStructure.TIM_Prescaler=(uint16_t) ((168000000/4) / 1000000) - 1;  //定时器分频 1000K
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM4
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	TIM_Cmd(TIM4,ENABLE); //使能定时器4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
//////////////////////////////////////////////////////////////////////////////////	 
//功能：设置跟随测试时的正弦波频率
//参数：frequency 单位 Hz
//返回值：无   							  
////////////////////////////////////////////////////////////////////////////////// 
void IntSetTestFrequency(int frequency)
{
	TIM4->ARR=(int)(1000000/50/frequency)-1;
}

//////////////////////////////////////////////////////////////////////////////////	 
//功能：设置控制间隔
//参数：interval 单位 us
//返回值：无   							  
////////////////////////////////////////////////////////////////////////////////// 
void IntSetCtrInterval(int interval)
{
	TIM4->ARR=(int)(interval)-1;	
}

void upload_data(u16 CMD, int data)
{
	u8 buf[11];
	u8 i;
	for(i=0;i<9;i++) buf[i]=0;//清0
	buf[0] = (u8)0xEE;
	buf[1] = (u8)0xCC;
	buf[2] = (u8)0xAA;
	buf[3] = (CMD>>8)&0XFF;
	buf[4] = (CMD)&0XFF;
	buf[5] = (data>>24)&0XFF;
	buf[6] = (data>>16)&0XFF;
	buf[7] = (data>>8)&0XFF;
	buf[8] = (data)&0XFF;
	for(i=3;i<9;i++) buf[9]+=buf[i];//计算校验和
	buf[10] = 0x0A;
	for(i=0;i<11;i++)usart1_send_char(buf[i]);	//发送数据到串口1 
} 

void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,c);   

} 

//串口2发送1个字符 
//c:要发送的字符
void usart2_send_char(u8 c)
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
    USART_SendData(USART2,c);   
} 