#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "pid.h"
#include "interuption.h"
#include "math.h"

int Sin[50];
int pitch = 0,roll = 0,yaw = 0; 		//欧拉角
int abs(int num)
{
	if(num < 0)
		return num;
	else
		return num;
}

//////////////////////////////////////////////////////////////////////////////////	 
//功能：中断服务函数
//参数：无
//返回值：无   							  
////////////////////////////////////////////////////////////////////////////////// 
int upload_index = 0;
int Division = 20, counter = 0;
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //溢出中断
	{
		//mpu_dmp_get_data(&pitch,&yaw,&roll);
		if(counter == Division)
		{
			//upload_data(CMD_FELLOW_TEST, Sin[upload_index]);
			upload_index++;
			if(upload_index >= 50)
				upload_index = 0;
			
			counter = 0;
		}
		counter++;
		LED0 = !LED0;
		upload_data(CMD_CLOCK, 0);
		upload_data(CMD_PITCH, pitch);
		//upload_data(CMD_YAW, yaw);
		usart2_send_char('p');
		
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //清除中断标志位
	}
}
//////////////////////////////////////////////////////////////////////////////////	 
//功能：初始化控制曲线
//参数：无
//返回值：无   							  
////////////////////////////////////////////////////////////////////////////////// 

void InitSin()
{
	for(int i = 0; i < 50; i++)
	{
		Sin[i] = (int)(AMPLITUDE*sin((float)(i*2*3.1415926)/50)*10);
	}
}


int main(void)
{ 
	int degree1 = 400, degree2 = 900;
	int direction = 1;
	int len;
	int delt;
	float pid_parametre_step = 0;
	int servo_degree_step = 0;
	int temp = 0;
	PID_AbsoluteType PID_Control1, PID_Control2;//定义PID算法的结构体
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(1000000);		//初始化串口波特率为500000
	MPU_Init();					//初始化MPU6050
	Servo_Init();
	InitSin();
	//Int_Init();
	LED_Init();
	Servo1_SetDegree(degree1);
	Servo2_SetDegree(degree2);
	LED0 = 1;
	while(mpu_dmp_init())
	{
 		delay_ms(4000);
		printf("hahaha");
	}
	Int_Init();

	
	PID_Control1.kp = 0.1;
	PID_Control1.ki = 0;
	PID_Control1.kd = 2;
	PID_Control1.errNow = 0;
	PID_Control1.ctrOut = 0;
	PID_Control1.ctrOutOld = 0;
	PID_Control1.errOld = 0;
	PID_Control1.errILim = 1000;
	
	PID_Control2.kp = 0.1;
	PID_Control2.ki = 0;
	PID_Control2.kd = 2;
	PID_Control2.errNow = 0;
	PID_Control2.ctrOut = 0;
	PID_Control1.ctrOutOld = 0;
	PID_Control1.errOld = 0;
	PID_Control1.errILim = 1000;
	
	IntSetCtrInterval(5000);
 	while(1)
	{
		if(mpu_dmp_get_data(&pitch,&yaw,&roll)==0)
		{ 
			//upload_data(0xFF, 0);
			//upload_data(CMD_YAW, yaw);
			//upload_data(CMD_ROLL, roll);
			//upload_data(CMD_DEGREE1, degree1);
			//upload_data(CMD_DEGREE2, degree2);
			//upload_data(CMD_KP, (int)(PID_Control1.kp*10000));
			//upload_data(CMD_KI, (int)(PID_Control1.ki*10000));
			//upload_data(CMD_KD, (int)(PID_Control1.kd*10000));
			//upload_data(CMD_CTROUT1, (int)PID_Control1.ctrOut);
		}
	
		
		if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			
			if(len == 1)
			{
				
				if(USART_RX_BUF[0] == CMD_UP)
				{
					degree1+=servo_degree_step;
					Servo1_SetDegree(degree1);
					//delay_ms(500);
				}
				else if(USART_RX_BUF[0] == CMD_DOWN)
				{
					degree1-=servo_degree_step;
					Servo1_SetDegree(degree1);
					//delay_ms(500);
				}
				else if(USART_RX_BUF[0] == CMD_LEFT)
				{
					degree2+=servo_degree_step;
					Servo2_SetDegree(degree2);
					//delay_ms(500);
				}
				else if(USART_RX_BUF[0] == CMD_RIGHT)
				{
					degree2-=servo_degree_step;
					Servo2_SetDegree(degree2);
				}
				else if(USART_RX_BUF[0] == CMD_POS_RESET)
				{
					degree1 = 500;
					Servo1_SetDegree(degree1);
					delay_ms(500);
				}
				else if(USART_RX_BUF[0] == CMD_KP_PLUS_1)
				{
					PID_Control1.kp += pid_parametre_step;
				}
				else if(USART_RX_BUF[0] == CMD_KP_MINUS_1)
				{
					PID_Control1.kp -= pid_parametre_step;
				}
				else if(USART_RX_BUF[0] == CMD_KI_PLUS_1)
				{
					PID_Control1.ki += pid_parametre_step;
				}
				else if(USART_RX_BUF[0] == CMD_KI_MINUS_1)
				{
					PID_Control1.ki -= pid_parametre_step;
				}
				else if(USART_RX_BUF[0] == CMD_KD_PLUS_1)
				{
					PID_Control1.kd += pid_parametre_step;
				}
				else if(USART_RX_BUF[0] == CMD_KD_MINUS_1)
				{
					PID_Control1.kd -= pid_parametre_step;
				}
				
				
			}
			else 
			{
				if(USART_RX_BUF[0] == CMD_PID_PRARMETRE_STEP_1)
				{
					pid_parametre_step = (float)(USART_RX_BUF[5] + USART_RX_BUF[4]*256 + USART_RX_BUF[3]*65536 + USART_RX_BUF[2]*16777216)/10000;
					servo_degree_step = (int)(pid_parametre_step*10);
				}
					
			}
				
			USART_RX_STA=0;
		}
		
		//PID 环节
		//目前最好的参数 
		//kp 0.1
		//ki 0
		//kd 2
		/*PID_Control1.errNow = -(yaw - DEST_YAW);
		PID_Control2.errNow = -(pitch - DEST_PITCH);
		PID_AbsoluteMode(&PID_Control1);
		PID_AbsoluteMode(&PID_Control2);
		if(abs((int)PID_Control1.ctrOut)>5)
			degree1+=(int)PID_Control1.ctrOut;
		if(abs((int)PID_Control2.ctrOut)>5)
			degree2+=(int)PID_Control2.ctrOut;*/

		if(degree1 < 200){
			degree1 = 200;
		}
		else if(degree1 > 1450){
			degree1 = 1450;
		}
		
		if(degree1 < 200){
			degree1 = 200;
		}
		else if(degree1 > 1450){
			degree1 = 1450;
		}
		
		Servo1_SetDegree(degree1);
		Servo2_SetDegree(degree2);
		//delay_ms(10);
		
		//PID_IncrementMode(&PID_Control2);
		//delay_ms(5000);
		//IntSetCtrInterval(1000);
	} 	
}


