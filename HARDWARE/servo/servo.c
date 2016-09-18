#include "sys.h"
#include "servo.h"
#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////	 
//功能：初始化舵机
//参数：无
//返回值：无   							  
////////////////////////////////////////////////////////////////////////////////// 
void Servo_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t CCR_Val = ZERO_DEGREE_PULSE_WIDTH*3;  //初始脉宽为1.5ms，舵机90度
	uint16_t PrescalerValue = 0;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* GPIOC Configuration: TIM3 CH1 (PC6), TIM3 CH2 (PC7), TIM3 CH3 (PC8) and TIM3 CH4 (PC9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3); 
	
	/* 这里将四个通道全部配置成50Hz 以便驱动舵机 */
	/* Compute the prescaler value 定时器的时钟设置到1000K*/
  PrescalerValue = (uint16_t) ((168000000 /2) / TIMER_CLOCK) - 1; 

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = TIMER_CLOCK/CONTROL_FREQUENCY - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1-Channel4 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);


  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_InternalClockConfig(TIM3);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
	
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

//使舵机转动角度
void Servo1_SetDegree(int degree)
{
	int cont = 0;
	cont = ZERO_DEGREE_PULSE_WIDTH + degree * CONTROL_RANGE / 1800;
	TIM_SetCompare1(TIM3,cont);
}

void Servo2_SetDegree(int degree)
{
	int cont = 0;
	cont = ZERO_DEGREE_PULSE_WIDTH + degree * CONTROL_RANGE / 1800;
	TIM_SetCompare2(TIM3,cont);
}

void Servo3_SetDegree(int degree)
{
	int cont = 0;
	cont = ZERO_DEGREE_PULSE_WIDTH + degree * CONTROL_RANGE / 1800;
	TIM_SetCompare3(TIM3,cont);
}

void Servo4_SetDegree(int degree)
{
	int cont = 0;
	cont = ZERO_DEGREE_PULSE_WIDTH + degree * CONTROL_RANGE / 1800;
	TIM_SetCompare4(TIM3,cont);
}
