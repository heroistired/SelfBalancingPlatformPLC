//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ��ж���صĿ��ƺͺ���
//��飺ʹ��TIM4
//���ߣ��»� �廪��ѧ���������Ӧ�õ��Ӽ���ϵ  
//���䣺heroistired@gmail.com OR h-che14@mails.stinghua.edu.cn								  
////////////////////////////////////////////////////////////////////////////////// 

#ifndef __INTERUPTION_H
#define __INTERUPTION_H	
#include "sys.h"
#define AMPLITUDE 30



void Int_Init();
void IntSetTestFrequency(int frequency);
void IntSetCtrInterval(int interval);
void InitSin();  
void upload_data(u16 CMD, int data);
void usart1_send_char(u8 c);

#endif
