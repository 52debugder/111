#include "ALL_DATA.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "mpu6050.h"
#include "usart.h"

#define SUCCESS 0
#undef FAILED
#define FAILED  1
/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
 
//uint8_t RC_rxData[32];
float CH[18];

//static float Pitch_Start,Roll_Start,Start_Remot_Flag = 1;
float Dis_Remot = 100,NatureX_Remot,NatureY_Remot; 
void remote_unlock(void);	
void RC_Analy(void)  
{
	
	
	//Sbus_Data_Count(); //遥控器接收数据
	//接收遥控的所有数据转移
	//roll控制杆
	Remote.roll = (CH[0]-216)/1.6+1000;
	Remote.roll = LIMIT(Remote.roll,1000,2000);
	//pitch控制杆
	Remote.pitch = 3000-((CH[1]-187)/1.6+1000); 
	Remote.pitch = LIMIT(Remote.pitch,1000,2000);
	//油门控制杆
	Remote.thr = 	(CH[2]-184)/1.6+1000;   
	Remote.thr = 	LIMIT(Remote.thr,1000,2000);
	//yaw控制杆
	Remote.yaw =  2000-((CH[3]-216)/1.6);  
	Remote.yaw =  LIMIT(Remote.yaw,1000,2000);
	//两个拨码开关
	Remote.AUX1 =  CH[5];   //遥控器正面右上角拨码开关    值：200/1800																		
	Remote.AUX2 =  CH[4];	//遥控器正面左上角拨码开关  	值：200/1000/1800
	
	if(Remote.AUX2 == 1800)//控制锁定
	{
		ALL_flag.unlock = 1;
		
		LED.status = AlwaysOn;
		PilotLED();
		
		Start_Flag = 1;
	}
	else
	{
		ALL_flag.unlock = 0;
		
		LED.status = AllFlashLight;
		PilotLED();
	}
	
	
//	Dis_Remot = (Remote.thr-1000)*0.2;
//	if(Remote.thr>1550||Remote.thr<1450)
//	{
//		Dis_Remot += (Remote.thr-1500)*0.0001;
//	}
//	if(Remote.pitch>1550||Remote.pitch<1450)
//	{
//		NatureX_Remot += -(Remote.pitch-1500)*0.0005;
//	}
//		
//	
//	
//	if(Remote.roll>1550||Remote.roll<1450)
//	{
//		NatureY_Remot += -(Remote.roll-1500)*0.0005;
//	}
	
}

/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
#include "imu.h"
void remote_unlock(void)     //状态机实现
{
	volatile static uint8_t status=WAITING_1; //设定初始状态
	static uint16_t cnt=0;     //软件计数器，用于给状态计时

	if(Remote.thr<1050 &&Remote.yaw<1200)                         //油门遥杆左下角锁定
	{
		status = EXIT_255;			//状态切换
	}
	
	switch(status)
	{
		case WAITING_1://等待解锁
			if(Remote.thr<1150)           //解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
			{			 
				 status = WAITING_2;				 
			}		
			break;
		case WAITING_2:
			if(Remote.thr>1600)          
			{		
				static uint8_t cnt = 0;
				cnt++;		
				if(cnt>5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
				{	
					cnt=0;
					status = WAITING_3;
				}
			}			
			break;
		case WAITING_3:
			if(Remote.thr<1100)          
			{			 
				 status = WAITING_4;				 
			}			
			break;			
		case WAITING_4:				//解锁前准备	               
			ALL_flag.unlock = 1;    //解锁标志位置1
			status = PROCESS_31;	//进入下一个状态
			LED.status = AlwaysOn;	//led常亮
			imu_rest();				//目测是陀螺仪参数初始化
		
			Start_Flag = 1;         //开始标志位拉起
			break;		
		case PROCESS_31:	//进入解锁状态
			
			if(Remote.thr<1020)
			{
				if(cnt++ > 3000)                                     // 油门遥杆处于最低9S自动上锁
				{								
					status = EXIT_255;								
				}
			}
			else if(!ALL_flag.unlock)                           //Other conditions lock 
			{
				status = EXIT_255;				
			}
			else					
				cnt = 0;
			break;
		case EXIT_255: //进入锁定
			LED.status = AllFlashLight;	//exit
			cnt = 0;
			LED.FlashTime = 100; //100*3ms		
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/

