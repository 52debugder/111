#include "all_data.h"
#include "imu.h"
#include "myMath.h"
#include <math.h>


Kalman KF;
float Accler_X = 0,Accler_Y = 0,Accler_Z = 0,R_Accler_X = 0,R_Accler_Y = 0,R_Accler_Z = 0,speed_Mpu = 0,Speed_Mpu02 = 0,AccX,AccY,speed_Mpu_X,speed_Mpu_Y;
static float NormAcc;

 
typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;
 Quaternion NumQ = {1, 0, 0, 0};
 
	 struct V{
				float x;
				float y;
				float z;
				};
	
volatile	 struct V GyroIntegError = {0}; 
	 
void imu_rest(void)
{
	NumQ.q0 =1;
	NumQ.q1 = 0;
	NumQ.q2 = 0;
	NumQ.q3 = 0;	
	GyroIntegError.x = 0;
	GyroIntegError.y = 0;
	GyroIntegError.z = 0;
	Angle.pitch = 0;
	Angle.roll = 0;
}


void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt) 
{		
	
volatile struct V Gravity,Acc,Gyro,AccGravity;
	static unsigned char Num;


	static  float KpDef = 0.5f ;
	static  float KiDef = 0.0001f;
//		static  float KiDef = 0.00001f;
	
	float q0_t,q1_t,q2_t,q3_t;
  //float NormAcc;	
	float NormQuat; 
	float HalfTime = dt * 0.5f;
	
	if(Num <= 240)
	{
		Num ++;
	}
	else
	{
		Accler_X = (((float)MPU6050.accX)/8192)*9.8*100;
		Accler_Y = (((float)MPU6050.accY)/8192)*9.8*100;
		Accler_Z = (((float)MPU6050.accZ-8192)/8192)*9.8*100;
		
//		R_Accler_X = Accler_X * cos(Angle.pitch/360*3.1415926);
//		R_Accler_Y = 
		
		speed_Mpu += Accler_Z*0.003;
		speed_Mpu_X  += Accler_X*0.003;
//		speed_Mpu_X *= cos(Angle.pitch/360*3.1415926);
		speed_Mpu_Y  += Accler_Y*0.003;
//		speed_Mpu_Y *= cos(Angle.roll/360*3.1415926);
		Speed_Mpu02 += Accler_Z*0.003;
		speed_Mpu_Y *= -1;
	}

	// 提取等效旋转矩阵中的重力分量 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	// 加速度归一化
 NormAcc = Q_rsqrt(squa(MPU6050.accX)+ squa(MPU6050.accY) +squa(MPU6050.accZ));
	
    Acc.x = pMpu->accX * NormAcc;
    Acc.y = pMpu->accY * NormAcc;
    Acc.z = pMpu->accZ * NormAcc;	
 	//向量差乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	//再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
	//角速度融合加速度积分补偿值
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	// 一阶龙格库塔法, 更新四元数

	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// 四元数归一化
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	
	

		// 四元数转欧拉角
	{
		 
			#ifdef	YAW_GYRO
			*(
		float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
			#else
				float yaw_G = pMpu->gyroZ * Gyro_G;
				if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //数据太小可以认为是干扰，不是偏航动作
				{
					pAngE->yaw  += yaw_G * dt;			
				}
			#endif
			pAngE->pitch  =  asin(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;						
		
			pAngE->roll	= atan2(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;	//PITCH 	
//			pAngE->roll += 0.4;
	}
}


void Kalman_Init()
{
    KF.Q = 0.225;			//过程噪声可以认为是0
    KF.R = 0.425;		//给一个较小的值，可以在debug中调节
    KF.Kg = 0;			
    KF.lastP = 1;		//lastP相当于上一次的值，初始值可以为1，不可以为0
    KF.x_hat = 0;		
}

float Kalman_Filter(Kalman *KF, float input)
{
    float output = 0, x_t;						//output为卡尔曼滤波计算值
    x_t = KF->x_hat;							//当前先验预测值 = 上一次最优值
    KF->nowP = KF->lastP + KF->Q;				//本次的协方差矩阵
    KF->Kg = KF->nowP / (KF->nowP + KF->R);		//卡尔曼增益系数计算
    output = x_t + KF->Kg*(input - x_t); 		//当前最优值
    KF->x_hat = output;							//更新最优值
    KF->lastP = (1 - KF->Kg) * KF->nowP;		//更新协方差矩阵
    return output;
}


/***************************************************END OF FILE***************************************************/




