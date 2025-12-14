#include "location.h"
#include <math.h>

int Location_X,Location_Y;   
unsigned char Start_Flag = 0;
float Nature_X,Nature_Y,Speed_Last,Speed_Now,Distance_Last,Speed_Hc_Sr04,Speed_HC_Sr04_Last,Speed_X,Speed_Y,Speed_Last_X,Speed_Last_Y,KF_DIS;
unsigned char Voliate;
#define Pai 3.1415926
extern float Laat_Pitch, Last_Roll,V_X,V_Y;
Kalman KF_Distance,KF_HCSr04_Speed,KF_Speed_X,KF_Speed_Y;
uint8_t Mis_Flage = 0;
float Distance;

void Get_Location(void)
{
	
	Distance = HAL_TIM_ReadCapturedValue(&htim4, 2)*0.017;    //输入捕获获得距离
	Distance = Distance*cos(Angle.pitch/360*Pai)*cos(Angle.roll/360*Pai);    //根据姿态将距离换算高度
//	KF_DIS = Kalman_Filter(&KF_Distance,Distance);
	if(Distance>=400 || Distance<=-400 /* ||(Distance - Distance_Last) >= 40 || (Distance - Distance_Last) <= -80*/)
	{
		//带通滤波，过高过低均认为杂波，本次测得高度即为上次测得高度
		Mis_Flage = 1;      //“跳过”标志位
							//如果本次获取到的数据不合理
							//用上次数据代替本次数据，
		Distance = Distance_Last;
	}
	Speed_Hc_Sr04 = (Distance-Distance_Last)/0.003;///0.009;  // Zv=dh/dt
	
	if(Mis_Flage == 1)   //如果获取到的超声波高度数据不合理，则认为计算出的速度数据也不合理，跳过本次数据
	{
		Mis_Flage = 0;
		Speed_Hc_Sr04 = Speed_HC_Sr04_Last;//本次世界z速度即为上次世界z速度
	}
	else
	{
		Distance_Last = Distance;    //本次测得高度合理，则将本次的高度设为上次高度，为下次准备
	}

//	Distance_Last = Distance;
	Speed_Hc_Sr04 = Kalman_Filter(&KF_HCSr04_Speed,Speed_Hc_Sr04);//将获取的世界z速度压入卡尔曼滤波器
	if(Speed_Hc_Sr04>=400 || Speed_Hc_Sr04<=-400)  //带通滤波
	{
		Speed_Hc_Sr04 = Speed_HC_Sr04_Last;
	}

	Speed_Now = (1-0.25)*(Speed_Last+speed_Mpu)+0.25*(Speed_Hc_Sr04);   //数据融合
																		 //本次超声波获取的速度、上次超声波获取的速度、陀螺仪获取的速度权重相加
																		 //拟合出实际世界z速度
	speed_Mpu = 0;     //陀螺仪测得速度清零
	Speed_HC_Sr04_Last = Speed_Hc_Sr04;      //本次超声波速度改为上次速度，为下次获取速度做准备
	Speed_Now = Kalman_Filter(&KF_Distance,Speed_Now);    //将实际世界z速度压入卡尔曼滤波器，
	Speed_Last = Speed_Now;     //将本次实际世界z速度设为上次世界z速度
	if(Distance != 0)
	{
		Start_Flag = 1;
	}
	
/**********************下面目测是光流定点部分代码*************************************/
//不过本鼠看不懂
	
	
	
//	Location_Y = (IIC_Read_One_Byte((0X31<<1),0X06)<<8)| IIC_Read_One_Byte((0X31<<1),0X07);
//	Location_X =  (IIC_Read_One_Byte((0X31<<1),0X08)<<8)| IIC_Read_One_Byte((0X31<<1),0X09);
//	Location_X *= -1;
//	
//	
//	if(speed_Mpu_X<0.1 && speed_Mpu_X> (-0.1))
//	{
//		speed_Mpu_X = 0;
//	}
//	Speed_Last_X = Speed_X;
//	Speed_X = (Speed_Last_X+speed_Mpu_X)*(1-0.25)+(Location_X*Distance/100)*0.25;
//	speed_Mpu_X = 0;
//	
//	Speed_X = Kalman_Filter(&KF_Speed_X,Speed_X);
//	
//	if(speed_Mpu_Y<0.1 && speed_Mpu_Y>(-0.1))
//	{
//		speed_Mpu_Y = 0;
//	}
//	Speed_Last_Y = Speed_Y;

//	Speed_Y =(Speed_Last_Y+speed_Mpu_Y)*(1-0.25)+(Location_Y*Distance/100)*0.25;
//	speed_Mpu_Y = 0;
//	
//	Speed_Y = Kalman_Filter(&KF_Speed_Y,Speed_Y);
//	
//	Voliate = 	IIC_Read_One_Byte((0X31<<1),0X0A);
//	Nature_X += Speed_X*0.003;//0.009;
//	Nature_Y += Speed_Y*0.003;//0.009;
//	
//	Laat_Pitch = Angle.pitch;
//	Last_Roll = Angle.roll;
}



//卡尔曼滤波器参数设定
void Kalman_Dis_Init()
{
    KF_Distance.Q = 2.795;			//过程噪声可以认为是0
    KF_Distance.R = 0.001;		//给一个较小的值，可以在debug中调节
    KF_Distance.Kg = 0;			
    KF_Distance.lastP = 1;		//lastP相当于上一次的值，初始值可以为1，不可以为0
    KF_Distance.x_hat = 0;		
	 
	KF_Speed_X.Q = 0.185;			//过程噪声可以认为是0
    KF_Speed_X.R = 0.465;		//给一个较小的值，可以在debug中调节
    KF_Speed_X.Kg = 0;			
    KF_Speed_X.lastP = 1;		//lastP相当于上一次的值，初始值可以为1，不可以为0
    KF_Speed_X.x_hat = 0;	
	
	KF_Speed_Y.Q = 0.185;			//过程噪声可以认为是0
    KF_Speed_Y.R = 0.465;		//给一个较小的值，可以在debug中调节
    KF_Speed_Y.Kg = 0;			
    KF_Speed_Y.lastP = 1;		//lastP相当于上一次的值，初始值可以为1，不可以为0
    KF_Speed_Y.x_hat = 0;	
	
	KF_HCSr04_Speed.Q = 0.205;			//过程噪声可以认为是0
    KF_HCSr04_Speed.R = 0.445;		//给一个较小的值，可以在debug中调节
    KF_HCSr04_Speed.Kg = 0;			
    KF_HCSr04_Speed.lastP = 1;		//lastP相当于上一次的值，初始值可以为1，不可以为0
    KF_HCSr04_Speed.x_hat = 0;		
}






