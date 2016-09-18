//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ�����������Զ�ģʽ�Ĵ���
//��飺ʹ�ö�ʱ��TIM3���������CH1-4��Ӧ���1-4 �Թ�ƽ̨�϶��1��2��˵�����sys.h
//���ߣ��»� �廪��ѧ���������Ӧ�õ��Ӽ���ϵ  
//���䣺heroistired@gmail.com OR h-che14@mails.stinghua.edu.cn								  
////////////////////////////////////////////////////////////////////////////////// 

#ifndef __SERVO_H
#define __SERVO_H	

#define CONTROL_FREQUENCY 400
#define TIMER_CLOCK 1000000
#define CONTROL_RANGE TIMER_CLOCK/10/50
#define ZERO_DEGREE_PULSE_WIDTH TIMER_CLOCK/50/40


//�������
void Servo_Init(void);
void Servo1_SetDegree(int degree);
void Servo2_SetDegree(int degree);
void Servo3_SetDegree(int degree);
void Servo4_SetDegree(int degree);
//�Զ�ģʽ�Ĵ���
void up(void);
void half_up(void);
void down(void);
void left(void);
void right(void);
void init1(void);
void init2(void);
void AutomaticMode(void);
#endif
