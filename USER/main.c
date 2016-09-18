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
int excitation_curve = 0; //��ǰʱ�̵ļ�������ֵ
int pitch = 0,roll = 0,yaw = 0; 		//ŷ����
int abs(int num)
{
	if(num < 0)
		return num;
	else
		return num;
}

//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ�����ͨ��Э����������λ���ϴ�����
//������fun:���� data:���ݻ����� len:data����Ч���ݸ���
//����ֵ����   							  
////////////////////////////////////////////////////////////////////////////////// 
void usart2_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[35];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+4]=0;	//У��������
	send_buf[0]=0XAA;	//֡ͷ
	send_buf[1]=0XAA;	//֡ͷ
	send_buf[2]=fun;	//������
	send_buf[3]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//��������
	for(i=0;i<len+4;i++)send_buf[len+4]+=send_buf[i];	//����У���	
	for(i=0;i<len+5;i++)usart2_send_char(send_buf[i]);	//�������ݵ�����1 
}

//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ��ϱ����ٶȴ��������ݺ�����������
//������aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//����ֵ����   							  
//////////////////////////////////////////////////////////////////////////////////
void mpu6050_send_acc_gyro(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[18]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	tbuf[12]=0;
	tbuf[13]=0;
	tbuf[14]=0;
	tbuf[15]=0;
	tbuf[16]=0;
	tbuf[17]=0;
	usart2_niming_report(0X02,tbuf,18);//�Զ���֡,0XA1
}	

//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ��ϱ���̬����
//������roll:����� pitch:������ yaw:����� 
//����ֵ����   							  
//////////////////////////////////////////////////////////////////////////////////
void usart2_report_imu(short roll,short pitch,short yaw)
{
	u8 tbuf[12]; 
	u8 i;
	for(i=0;i<12;i++)tbuf[i]=0;//��0
	
	tbuf[0]=(roll>>8)&0XFF;
	tbuf[1]=roll&0XFF;
	tbuf[2]=(pitch>>8)&0XFF;
	tbuf[3]=pitch&0XFF;
	tbuf[4]=(yaw>>8)&0XFF;
	tbuf[5]=yaw&0XFF;
	tbuf[6]=0;
	tbuf[7]=0;
	tbuf[8]=0;
	tbuf[9]=0;
	tbuf[10]=1;
	tbuf[11]=1;
	usart2_niming_report(0X01,tbuf,12);//�ɿ���ʾ֡,0XAF
} 

//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ��ϱ��������ߺ�ʱ��
//������curve���������� clock��ʱ�� 
//����ֵ����   							  
////////////////////////////////////////////////////////////////////////////////// 

void usart2_report_excitation_curve_clock(int curve, u16 clock)
{
	u8 tbuf[6]; 
	u8 i;
	for(i=0;i<6;i++)tbuf[i]=0;//��0
	tbuf[0]=(curve>>24)&0XFF;
	tbuf[1]=(curve>>16)&0XFF;
	tbuf[2]=(curve>>8)&0XFF;
	tbuf[3]=curve&0XFF;
	tbuf[4]=(clock>>8)&0XFF;
	tbuf[5]=clock&0XFF;
	usart2_niming_report(0X07,tbuf,6);//�ɿ���ʾ֡,0XAF
} 


//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ��жϷ�����
//��������
//����ֵ����   							  
////////////////////////////////////////////////////////////////////////////////// 
int upload_index = 0;
int Division = 1, counter = 0;
u16 clock = 0;
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) //����ж�
	{
		//ʱ���źŷ�ת
		if(!clock)
			clock = 1000;
		else 
			clock = 0;
		//���Ҽ����ź�
		if(counter == Division)
		{
			excitation_curve = Sin[upload_index];
			upload_index++;
			if(upload_index >= 50)
				upload_index = 0;
			
			counter = 0;
		}
		counter++;
		//upload_data(CMD_CLOCK, 0);
		//upload_data(CMD_PITCH, pitch);
		//upload_data(CMD_YAW, yaw);
		//usart2_send_char('p');
		//upload_data(0xFF, 0);
			//upload_data(CMD_YAW, yaw);
			//upload_data(CMD_ROLL, roll);
			//upload_data(CMD_DEGREE1, degree1);
			//upload_data(CMD_DEGREE2, degree2);
			//upload_data(CMD_KP, (int)(PID_Control1.kp*10000));
			//upload_data(CMD_KI, (int)(PID_Control1.ki*10000));
			//upload_data(CMD_KD, (int)(PID_Control1.kd*10000));
			//upload_data(CMD_CTROUT1, (int)PID_Control1.ctrOut);
		mpu_dmp_get_data(&pitch,&yaw,&roll);
		usart2_report_excitation_curve_clock(excitation_curve*10, clock);
		usart2_report_imu((int)(roll*10),(int)(pitch*10),(int)(yaw*10));
		
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //����жϱ�־λ
	}
}
//////////////////////////////////////////////////////////////////////////////////	 
//���ܣ���ʼ����������
//��������
//����ֵ����   							  
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
	PID_AbsoluteType PID_Control1, PID_Control2;//����PID�㷨�Ľṹ��
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ500000
	MPU_Init();					//��ʼ��MPU6050
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
	
	IntSetCtrInterval(100000);
 	while(1)
	{
		/*if(mpu_dmp_get_data(&pitch,&yaw,&roll)==0)
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
		}*/
	
		
		if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			
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
		
		//PID ����
		//Ŀǰ��õĲ��� 
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

		/*if(degree1 < 200){
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
		//IntSetCtrInterval(1000);*/
	} 
}


