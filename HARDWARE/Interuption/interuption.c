#include "sys.h"
#include "interuption.h"
#include "delay.h"
#include "math.h"
#include "led.h"



//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ��жϳ�ʼ��
//��������
//����ֵ����   							  
////////////////////////////////////////////////////////////////////////////////// 
void Int_Init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///ʹ��TIM4ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1; 	//�Զ���װ��ֵ ��ʼ���� 2ms �ж�һ��
	TIM_TimeBaseInitStructure.TIM_Prescaler=(uint16_t) ((168000000/4) / 1000000) - 1;  //��ʱ����Ƶ 1000K
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=0; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//��ʼ��TIM4
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //����ʱ��4�����ж�
	TIM_Cmd(TIM4,ENABLE); //ʹ�ܶ�ʱ��4
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ����ø������ʱ�����Ҳ�Ƶ��
//������frequency ��λ Hz
//����ֵ����   							  
////////////////////////////////////////////////////////////////////////////////// 
void IntSetTestFrequency(int frequency)
{
	TIM4->ARR=(int)(1000000/50/frequency)-1;
}

//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ����ÿ��Ƽ��
//������interval ��λ us
//����ֵ����   							  
////////////////////////////////////////////////////////////////////////////////// 
void IntSetCtrInterval(int interval)
{
	TIM4->ARR=(int)(interval)-1;	
}

void upload_data(u16 CMD, int data)
{
	u8 buf[11];
	u8 i;
	for(i=0;i<9;i++) buf[i]=0;//��0
	buf[0] = (u8)0xEE;
	buf[1] = (u8)0xCC;
	buf[2] = (u8)0xAA;
	buf[3] = (CMD>>8)&0XFF;
	buf[4] = (CMD)&0XFF;
	buf[5] = (data>>24)&0XFF;
	buf[6] = (data>>16)&0XFF;
	buf[7] = (data>>8)&0XFF;
	buf[8] = (data)&0XFF;
	for(i=3;i<9;i++) buf[9]+=buf[i];//����У���
	buf[10] = 0x0A;
	for(i=0;i<11;i++)usart1_send_char(buf[i]);	//�������ݵ�����1 
} 

void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); 
    USART_SendData(USART1,c);   

} 

//����2����1���ַ� 
//c:Ҫ���͵��ַ�
void usart2_send_char(u8 c)
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
    USART_SendData(USART2,c);   
} 