//////////////////////////////////////////////////////////////////////////////////	 
//功能：中断相关的控制和函数
//简介：使用TIM4
//作者：陈欢 清华大学电机工程与应用电子技术系  
//邮箱：heroistired@gmail.com OR h-che14@mails.stinghua.edu.cn								  
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
