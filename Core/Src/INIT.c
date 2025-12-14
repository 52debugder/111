#include "ALL_DEFINE.h"


volatile uint32_t SysTick_count; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_AngE Angle;    //当前角度姿态值
_st_Remote Remote; //遥控通道值


_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等

PidObject pidNatureX;//物理位置pid数据
PidObject pidNatureY;


PidObject pidLocationX;//位置pid数据
PidObject pidLocationY;
PidObject pidHigh;
PidObject pidHigh_Speed;


PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
PidObject pidRoll;
PidObject pidYaw;


void pid_param_Init(void); //PID控制参数初始化，改写PID并不会保存数据，请调试完成后直接在程序里更改 再烧录到飞控
void pid_limit_Init(PidObject *pid,float IntegLimitHigh,float IntegLimitLow,float OutLimitHigh,float OutLimitLow);

int16_t motor_PWM_Value[4];    //四个电机的pwm值，利用电调可以直接使用pwm控制无刷电机
								//全局变量，初始值为0

void ALL_Init(void)
{
	ALL_flag.unlock = 0;     //解锁标志位置0,表示飞行器目前已锁定


	HAL_I2C_MspInit(&hi2c2);             //I2C初始化
	
	pid_param_Init();       //PID参数初始化
	
	
	MPU6050_Init();              //MPU6050初始化

//	delay_ms(10000);
	MpuGetOffset();      //6050校准
						//目测方法是上电一段时间内不直接读取姿态信息
						//对这段时间内的传感器获取的数据采样取平均值，估计出漂移量
						//正式获取姿态信息时将估计出的漂移量融合进去进行校正
	
	//USART1_Config();  //备用串口    
	//HAL_UART_MspDeInit(&huart3); 	//串口3
	Kalman_Init();		//卡尔曼滤波器初始化
	Kalman_Dis_Init();

	UT_Init();   //超声波测距输入捕获初始化
	
	HAL_TIM_MspPostInit(&htim2);			//4路PWM初始化
	HAL_TIM_MspPostInit(&htim4);     	//LED PWM初始化
	
	HAL_TIM_MspPostInit(&htim3); 					//系统工作周期初始化 
	

	
}

void pid_param_Init(void)//PID参数初始化
{
	pidHigh.OutLimitHigh = 200;      //目测高度环输出限幅
	pidHigh.OutLimitLow = -200;
	
	pidRateX.kp = 0.475f;			//x轴角速度环
	pidRateX.kd = 0.003f;
	
	pidRateY.kp = 0.475f;			//y轴角速度环
	pidRateY.kd = 0.003f;
	
	pidRateZ.kp = 0.475f;			//z轴角速度环
	pidRateZ.kd = 0.003f;
		
	pidPitch.kp = 6.75f;			//俯仰角角度环
	pidPitch.ki = 6.05f;
	pidPitch.kd = 0.0002f;
	
	pidRoll.kp = 6.75f;				//翻滚角角度环
	pidRoll.ki = 6.05f;
	pidRoll.kd  = 0.0002f;
	
	pidYaw.kp = 6.75f;				//偏航角角度环
	pidYaw.ki = 6.05f;
	pidYaw.kd = 0.0002f;
	
//	pidYaw.kp = 10.5f;	
////	pidYaw.ki = 1.85f;
//	pidYaw.kd   = 0.001f;
	
	
	
	/*Hight_9ms*/
//	pidHigh.kp = 1.6735;
//	pidHigh.ki = 0.00;
//	pidHigh.kd = 0.0012;

//	pidHigh_Speed.kp = 2.6525;
//	pidHigh_Speed.ki = 1.055;
//	pidHigh_Speed.kd = 0.012;
	
	/*Hight_3ms*/
	pidHigh.kp = 0.7535;                 //高度环
	pidHigh.ki = 0.00;
	pidHigh.kd = 0.00650;

	pidHigh_Speed.kp = 1.3525;			//世界z速度环
	pidHigh_Speed.ki = 1.055;
	pidHigh_Speed.kd = 0.004;
	
//	pidHigh_Speed.kp = 1.4525;
//	pidHigh_Speed.ki = 1.455;
//	pidHigh_Speed.kd = 0.010;
	
	
	
	pidNatureX.kp = 2.255;
//	pidNatureX.ki = 0.05;
	pidNatureX.kd = 0.0012;
	
	pidNatureY.kp = 2.255;
//	pidNatureY.ki = 0.05;
	pidNatureY.kd = 0.0012; 
	
	pidLocationX.kp = 0.14875;
	pidLocationX.ki = 0.00;
	pidLocationX.kd = 0.000450;
	
	pidLocationY.kp = 0.14875;
	pidLocationY.ki = 0.00;
	pidLocationY.kd = 0.000450;
}

void pid_limit_Init(PidObject *pid,float IntegLimitHigh,float IntegLimitLow,float OutLimitHigh,float OutLimitLow)
{
	pid->IntegLimitHigh = IntegLimitHigh;
	pid->IntegLimitLow = IntegLimitLow;
	pid->OutLimitHigh = OutLimitHigh;
	pid->IntegLimitLow = OutLimitLow;
}




