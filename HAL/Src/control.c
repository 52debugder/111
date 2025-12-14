/*
	机头与电机示意图	
					 机头(x+)
					   
				  M2   ↑    M1
					\   |   /
					 \  |  /
					  \ | /
			    ————————-————————>y+	
					  / | \
					 /  |  \
					/   |   \
				  M4    |    M3
				  
	1. M2 M3电机逆时针旋转，M1 M4电机顺时针旋转
	2. X:是MPU6050的 X 轴，Y:是MPU6050的 Y 轴，Z轴正方向垂直 X-Y 面，竖直向上
	3. 绕 X 轴旋转为ROLL 角 
	   绕 Y 轴旋转为 PITCH 角 
	   绕 Z 轴旋转为 YAW  角
*/



#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT 0
int FlowDown_Flag = 7999;
int pidHigh_desired = 150;
int NatureX_desired[31] = {0, 50,100,150,150,150,150,150,100, 50,  0,  0,  0, 50,100,150,150,150,100,100,100,100,100,100,100,100,100,100,100,100,100};
int NatureY_desired[31] = {0,  0,  0,  0,  0, 50,100,100,100,100,100,150,150,150,150,150,150,150,100,100,100,100,100,100,100,100,100,100,100,100,100};

int Nature_Flage = 0;
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw,&pidHigh,&pidLocationX,&pidLocationY,&pidNatureX,&pidNatureY,&pidHigh_Speed
    //结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据，新手明白这句话的作用就可以了
};
PidObject *(pPidObject_L[])={&pidNatureX,&pidNatureY,&pidLocationX,&pidLocationY};
/**************************************************************
 *  flight control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
float roll_pitch_ratio = 0.05f;       //应该是将摇杆值与实际对应的角度的一个换算比例值
float yaw_ratio =  0.3f;	

//飞行器遥控控制-remote control 
void FlightPidControl_RC(float dt)
{
	volatile static uint8_t status=WAITING_1;
	
	//
	//if(Remote.yaw>1820)
	if(Remote.yaw>1700)
	{
		pidYaw.desired -= yaw_ratio;	
	}
	//else if(Remote.yaw <1180)
	else if(Remote.yaw <1300)	
	{
		pidYaw.desired += yaw_ratio;	
	}	
	
	
	switch(status)
	{		
		case WAITING_1: //等待解锁
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //准备进入控制
			pidRest(pPidObject,11); //批量复位PID数据，防止上次遗留的数据影响本次控制

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //锁定偏航角
		
			status = PROCESS_31;
			
			
			break;			
		case PROCESS_31: //正式进入控制
			if(Angle.pitch<-50||Angle.pitch>50||Angle.roll<-50||Angle.roll>50)//倾斜检测，大角度判定为意外情况，则紧急上锁	
					if(Remote.thr>1200)//当油门的很低时不做倾斜检测
						ALL_flag.unlock = EMERGENT;//打入紧急情况
					
			pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;	 //将遥杆值作为飞行角度的期望值
			pidRoll.desired  = -(Remote.roll-1500)*roll_pitch_ratio;
					
			pidRateX.measured = MPU6050.gyroX * Gyro_G; //内环测量值 角度/秒
			pidRateY.measured = MPU6050.gyroY * Gyro_G;
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
		
			pidPitch.measured = Angle.pitch; //外环测量值 单位：角度
			pidRoll.measured  = Angle.roll;
			pidYaw.measured   = Angle.yaw;
					
		 	pidUpdate(&pidRoll,dt);    //调用PID处理函数来处理外环	横滚角PID		
			pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
			pidUpdate(&pidRateX,dt);  //再调用内环

		 	pidUpdate(&pidPitch,dt);    //调用PID处理函数来处理外环	俯仰角PID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //再调用内环

			CascadePID(&pidRateZ,&pidYaw,dt);	//也可以直接调用串级PID函数来处理，作用和前面两段代码一样
			
			break;
		case EXIT_255:  //退出控制
			pidRest(pPidObject,9);
			status = WAITING_1;//返回等待解锁
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
}


//飞行器自动控制-Auto
uint8_t Fall_down = 0;
void FlightPidControl_Auto(float dt)
{
//	static unsigned char Start_Flag = 1;
	volatile static uint8_t status=WAITING_1;
//	const float roll_pitch_ratio = 0.12f;
//	const float yaw_ratio =   0.3f;	
	
	//status =  PROCESS_31;   //此行代码为飞行器正式进去控制的条件
							  //取消注释飞行器上电自动解锁，但不会直接起飞
							  //需要在MotorControl（）函数中再次解锁才会起飞
	
	switch(status)
	{		
		case WAITING_1: //等待解锁
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //准备进入控制
			pidRest(pPidObject,11); //批量复位PID数据，防止上次遗留的数据影响本次控制

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //锁定偏航角
		
			status = PROCESS_31;
		
			break;
		
		case PROCESS_31: //正式进入控制
			if(Angle.pitch<-50||Angle.pitch>50||Angle.roll<-50||Angle.roll>50)//倾斜检测，大角度判定为意外情况，则紧急上锁	
					if(Remote.thr>1200)//当油门的值很低时不做倾斜检测
						ALL_flag.unlock = EMERGENT;//打入紧急情况
//*****************************************************************************
//位置环pid处理
						
			if(Remote.AUX1 == 200)
			{
				pidHigh_desired = 150;
				pidHigh.desired = pidHigh_desired;
			}
			
			pidHigh.measured = Distance;
			
			pidUpdate(&pidHigh,dt);
//******************************************************************************					
//速度环pid处理

			pidHigh_Speed.desired = LIMIT(pidHigh.out,-200,200);
			

			pidHigh_Speed.measured = Speed_Now;
			
			pidUpdate(&pidHigh_Speed,dt);


//******************************************************************************
//角度环Pid处理			此时处于定高模式
			if(Remote.yaw>1700)
			{
				pidYaw.desired -= yaw_ratio;	
			}
			//else if(Remote.yaw <1180)
			else if(Remote.yaw <1300)	
			{
				pidYaw.desired += yaw_ratio;	
			}	
			pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;	 //将遥杆值作为飞行角度的期望值
			pidRoll.desired  = -(Remote.roll-1500)*roll_pitch_ratio;

			
			pidPitch.measured = Angle.pitch; //外环测量值 单位：角度
			pidRoll.measured = Angle.roll;
			pidYaw.measured = Angle.yaw;
			
			
			pidUpdate(&pidRoll,dt); 
			pidUpdate(&pidPitch,dt); 
			pidUpdate(&pidYaw,dt); 	
//*******************************************************************************
//角速度环pid处理
			pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
			pidRateY.desired = pidPitch.out;  
			pidRateZ.desired = pidYaw.out;
			
			pidRateX.measured = MPU6050.gyroX * Gyro_G;
			pidRateY.measured = MPU6050.gyroY * Gyro_G;
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G;
			
			pidUpdate(&pidRateX,dt);
			pidUpdate(&pidRateY,dt);
			pidUpdate(&pidRateZ,dt);
//*******************************************************************************	
			break;
			
		case EXIT_255:  //退出控制
			pidRest(pPidObject,11);
			status = WAITING_1;//返回等待解锁
			break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
}


//四个电机的占空比
#define MOTOR1 motor_PWM_Value[0] 
#define MOTOR2 motor_PWM_Value[1] 
#define MOTOR3 motor_PWM_Value[2] 
#define MOTOR4 motor_PWM_Value[3] 

//不知道是啥
uint16_t low_thr_cnt_quiet;
uint16_t low_thr_cnt;


//电机控制
void MotorControl(void)
{	
	
				if(Remote.AUX2 == 1800)//控制解锁，获取摇杆油门值
				{
					if(Remote.AUX1 == 1800)
					{
						MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(pidHigh_Speed.out+450,0,650);	
					}
					else
					{
						MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT((Remote.thr-1000),0,800); //留100给姿态控制
					}
				}
				else
				{
					//
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 =0;
				}
				
	
				
		MOTOR1 += (- pidRateX.out - pidRateY.out + pidRateZ.out);//; 姿态输出分配给各个电机的控制量
		MOTOR2 += (+ pidRateX.out - pidRateY.out - pidRateZ.out);//;
		MOTOR3 += (- pidRateX.out + pidRateY.out - pidRateZ.out);
		MOTOR4 += (+ pidRateX.out + pidRateY.out + pidRateZ.out);//;	

		MOTOR1 += 1000;
		MOTOR2 += 1000;
		MOTOR3 += 1000;
		MOTOR4 += 1000;

		if (Start_Flag == 0 || Remote.AUX2 == 200)
		{
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 1000;  //如果锁定，则电机输出都为0
			pidRest(pPidObject,12);//pid复位防止数据影响
		}
		
		TIM2->CCR1 = LIMIT(MOTOR1,1000,2000);  //更新PWM并限幅
		TIM2->CCR2 = LIMIT(MOTOR2,1000,2000);
		TIM2->CCR3 = LIMIT(MOTOR3,1000,2000);
		TIM2->CCR4 = LIMIT(MOTOR4,1000,2000);
}

/************************************END OF FILE********************************************/ 
