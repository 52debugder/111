#ifndef _ALL_USER_DATA_H_
#define _ALL_USER_DATA_H_

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       long long int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       long long uint64_t;


//#define NULL 0
extern volatile uint32_t SysTick_count;
extern uint8_t Serial_TxPacket[100];			//FF 01 02 03 04 FE

typedef struct{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
}_st_Mpu;




typedef struct{
	float roll;
	float pitch;
	float yaw;
}_st_AngE;



typedef struct
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t thr;
	uint16_t yaw;
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;	
}_st_Remote;



typedef volatile struct
{
	float desired;     //< set point
	float offset;      //
	float prevError;    //< previous error
	float integ;        //< integral
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float IntegLimitHigh;       //< integral limit
	float IntegLimitLow;
	float measured;
	float out;
	float OutLimitHigh;
	float OutLimitLow;
}PidObject;


typedef volatile struct
{
	uint8_t unlock;
	

}_st_ALL_flag;


extern float LX,LY,Dis,NX,NY;
extern PidObject pidLocationX;
extern PidObject pidLocationY;
extern PidObject pidHigh;
extern PidObject pidHigh_Speed;
extern PidObject pidNatureX;//物理位置pid数据
extern PidObject pidNatureY;


extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_AngE Angle;

extern float Dis_Remot,NatureX_Remot,NatureY_Remot; 


extern _st_ALL_flag ALL_flag;


extern	PidObject pidRateX;
extern	PidObject pidRateY;
extern	PidObject pidRateZ;

extern	PidObject pidPitch;
extern	PidObject pidRoll;
extern	PidObject pidYaw;
extern float Distance;
extern int Location_X,Location_Y;

extern float speed_Mpu,Speed_Mpu02,speed_Mpu_X,speed_Mpu_Y,Speed_X,Speed_Y;

extern float Speed_Last,Speed_Now,Distance_Last,Speed_Hc_Sr04,KF_DIS;

extern int16_t motor_PWM_Value[4];

extern float Nature_X,Nature_Y;

extern unsigned char Start_Flag;

extern unsigned char Voliate;

extern int pidHigh_desired;

extern int Nature_Flage;

typedef struct
{
    float lastP;		//上次的协方差
    float nowP;			//本次的协方差
    float x_hat;		//卡尔曼滤波的计算值，即为后验最优值
    float Kg;			//卡尔曼增益系数
    float Q;			//过程噪声
    float R;			//测量噪声
}Kalman;


#endif

