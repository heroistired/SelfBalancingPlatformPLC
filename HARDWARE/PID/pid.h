#ifndef __FUNTION_H
#define __FUNTION_H
#include "sys.h"	 

/*绝对式PID算法，接口参数结构类型*/
typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 float errILim;//误差积分上限
 
 float errNow;//当前的误差
 float ctrOut;//控制量输出
 float ctrOutOld;
 
 /*PID算法内部变量，其值不能修改*/
 float errOld;
 float errP;
 float errI;
 float errD;
 
}PID_AbsoluteType;


/*增量式PID算法，接口参数结构类型*/
typedef struct 
{
 /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
 float kp;     //比例系数
 float ki;     //积分系数
 float kd;     //微分系数
 
 float errNow; //当前的误差
 float dCtrOut;//控制增量输出
 float  ctrOut;//控制输出
 
 /*PID算法内部变量，其值不能修改*/
 float errOld1;
 float errOld2;
 
}PID_IncrementType;


void PID_AbsoluteMode(PID_AbsoluteType* PID);

void PID_IncrementMode(PID_IncrementType* PID);

#endif







 