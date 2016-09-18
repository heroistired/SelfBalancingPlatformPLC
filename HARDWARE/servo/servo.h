//////////////////////////////////////////////////////////////////////////////////	 
//功能：舵机驱动及自动模式的代码
//简介：使用定时器TIM3驱动舵机，CH1-4对应舵机1-4 迷宫平台上舵机1、2的说明详见sys.h
//作者：陈欢 清华大学电机工程与应用电子技术系  
//邮箱：heroistired@gmail.com OR h-che14@mails.stinghua.edu.cn								  
////////////////////////////////////////////////////////////////////////////////// 

#ifndef __SERVO_H
#define __SERVO_H	

#define CONTROL_FREQUENCY 400
#define TIMER_CLOCK 1000000
#define CONTROL_RANGE TIMER_CLOCK/10/50
#define ZERO_DEGREE_PULSE_WIDTH TIMER_CLOCK/50/40


//舵机驱动
void Servo_Init(void);
void Servo1_SetDegree(int degree);
void Servo2_SetDegree(int degree);
void Servo3_SetDegree(int degree);
void Servo4_SetDegree(int degree);
//自动模式的代码
void up(void);
void half_up(void);
void down(void);
void left(void);
void right(void);
void init1(void);
void init2(void);
void AutomaticMode(void);
#endif
