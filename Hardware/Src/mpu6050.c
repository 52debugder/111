/*******************************************************************
 *MPU6050
 *@brief 
 *@brief 
 *@time  2016.1.8
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 *非授权使用人员，禁止使用。禁止传阅，违者一经发现，侵权处理。
 ******************************************************************/
#include "ALL_DATA.h"
#include "mpu6050.h"
#include "I2C.h"
#include "filter.h"
#include <string.h>
#include "LED.h"
#include "myMath.h"
#include "kalman.h"
#include "flash.h"



#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIGL			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_ADDRESS	0x3B
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define GYRO_ADDRESS  0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//IIC地址寄存器(默认数值0x68，只读)
#define MPU6050_PRODUCT_ID 0x68
#define MPU6052C_PRODUCT_ID 0x72

//#define   MPU6050_is_DRY()      GPIO_ReadOutBit(HT_GPIOC, GPIO_PIN_0)//IRQ主机数据输入
	#ifdef  	USE_I2C_HARDWARE
		
		#define MPU6050_ADDRESS 0xD0//0x68
	#else
		#define  MPU6050_ADDRESS 0xD0   //IIC写入时的地址字节数据，+1为读取
	#endif



int16_t MpuOffset[6] = {0};

static volatile int16_t *pMpu = (int16_t *)&MPU6050;

static int16_t Mpu6050Addr = 0x69;
MPU6050DATATYPE Mpu6050_Data;

int8_t Sensor_I2C2_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *oData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Read(&hi2c2,(DevAddr<<1)|0,MemAddr,1,oData,DataLen,1000);
}

int8_t Sensor_I2C2_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *iData, uint8_t DataLen)
{
	return HAL_I2C_Mem_Write(&hi2c2,(DevAddr<<1)|1,MemAddr,1,iData,DataLen,1000);
}

int16_t Sensor_I2C2_Serch(void)
{
	for(uint8_t i = 1; i < 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c2, i, 1, 1000) == HAL_OK)
		{
			Mpu6050Addr = i>>1;
			return i>>1;
		}
	}
	return -1; // ????
}

int8_t MPU6050_Init()
{
	uint8_t check;
	check = 0x80;
	Sensor_I2C2_Write(Mpu6050Addr,PWR_MGMT_1,&check, 1); 	    // ??
	HAL_Delay(100);
	check = 0x00;
	Sensor_I2C2_Write(Mpu6050Addr,PWR_MGMT_1,&check, 1); 	    // ??
	
	
	if(Sensor_I2C2_Serch() != -1) // ????? ?????
	{	
		check = 0x07;	
		Sensor_I2C2_Write(Mpu6050Addr,SMPLRT_DIV,&check, 1);	    // 1Khz???
		check = 0x00;
		Sensor_I2C2_Write(Mpu6050Addr,ACCEL_CONFIG,&check, 1);	 	// ?????
		check = 0x00;
		Sensor_I2C2_Write(Mpu6050Addr,GYRO_CONFIG,&check, 1);		// ????
		HAL_Delay(100);
		return 0;
	}
	return -1;
}

void MPU6050_Read_Accel(void)
{
	uint8_t Read_Buf[6];
	
	Mpu6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Accel_X = Mpu6050_Data.Accel_X / 16384.0f;
	Mpu6050_Data.Accel_Y = Mpu6050_Data.Accel_Y / 16384.0f;
	Mpu6050_Data.Accel_Z = Mpu6050_Data.Accel_Z / 16384.0f;
	
}
void MPU6050_Read_Gyro(void)
{
	uint8_t Read_Buf[6];
	
	// ????????X? - ??X? - ??Y?? - ??Y?? - ??Z?? - ??Z??
	if(HAL_OK != Sensor_I2C2_Read(Mpu6050Addr, GYRO_XOUT_H, Read_Buf, 6))
	{
		return;
	}		
	
	Mpu6050_Data.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Gyro_X = Mpu6050_Data.Gyro_X / 131.0f;
	Mpu6050_Data.Gyro_Y = Mpu6050_Data.Gyro_Y / 131.0f;
	Mpu6050_Data.Gyro_Z = Mpu6050_Data.Gyro_Z / 131.0f;
	
}
void MPU6050_Read_Temp(void)
{
    uint8_t Read_Buf[2];
	int ret = Sensor_I2C2_Read(Mpu6050Addr, TEMP_OUT_H, Read_Buf, 2);
	if(HAL_OK != ret) 
	{
		return;
	}
	
	Mpu6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	
	Mpu6050_Data.Temp = 36.53f + (Mpu6050_Data.Temp / 340.0f);
}


///****************************************************************************************
//*@brief  
//*@brief   
//*@param[in]
//*****************************************************************************************/
//int8_t mpu6050_rest(void)
//{
//	if(IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80) == FAILED)
//		return FAILED;	//复位
//	delay_ms(20);
//	return SUCCESS;
//}
///****************************************************************************************
//*@brief   
//*@brief  
//*@param[in]
//*****************************************************************************************/
//int8_t MpuInit(void) //初始化
//{
//	uint8_t date = SUCCESS;
//	do
//	{
//	date = HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, PWR_MGMT_1, 0x80);	//复位
//	delay_ms(30);
//	date += HAL_I2C_Mem_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x02); //陀螺仪采样率，0x00(500Hz)
//	date += HAL_I2C_Mem_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x03);	//设置设备时钟源，陀螺仪Z轴
//	date += HAL_I2C_Mem_Write(MPU6050_ADDRESS, CONFIGL, 0x03);   //低通滤波频率，0x03(42Hz)
//	date += HAL_I2C_Mem_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);//+-2000deg/s
//	date += HAL_I2C_Mem_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);//+-4G
//	}
//	while(date != SUCCESS);
//	date = IIC_Read_One_Byte(MPU6050_ADDRESS, 0x75);
//	if(date!= MPU6050_PRODUCT_ID)
//		return FAILED;		
//	
//	FLASH_read(MpuOffset,6);//从mcu的FLASH中读取MPU6050的水平静止标定校准值
//	return SUCCESS;
//}
///****************************************************************************************
//*@brief    
//*@brief   
//*@param[in]
//*****************************************************************************************/

//#define  Acc_Read() IIC_read_Bytes(MPU6050_ADDRESS, 0X3B,buffer,6)
//#define  Gyro_Read() IIC_read_Bytes(MPU6050_ADDRESS, 0x43,&buffer[6],6)

void MpuGetData(void) //读取陀螺仪数据加滤波
{
	  uint8_t i;
    uint8_t buffer[12];

		
	  MPU6050_Read_Accel();//去读加速度
		MPU6050_Read_Gyro();//读取角速度
	
	
		for(i=0;i<6;i++)
		{
			pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1])-MpuOffset[i];		//整合为16bit，并减去水平静止校准值
			if(i < 3)//以下对加速度做卡尔曼滤波
			{
				{
					static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
					kalman_1(&ekf[i],(float)pMpu[i]);  //一维卡尔曼
					pMpu[i] = (int16_t)ekf[i].out;
				}
		}
		if(i > 2)//以下对角速度做一阶低通滤波
		{	
			uint8_t k=i-3;
			const float factor = 0.15f;  //滤波因素			
			static float tBuff[3];		

			pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;                
		}
	}
}

/****************************************************************************************
*@brief   get mpu offset
*@brief   initial and cmd call this
*@param[in]
*****************************************************************************************/
void MpuGetOffset(void) //校准
{
	int32_t buffer[6]={0};
	int16_t i;  
	uint8_t k=30;
	const int8_t MAX_GYRO_QUIET = 5;
	const int8_t MIN_GYRO_QUIET = -5;	
/*           wait for calm down    	                                                          */
	int16_t LastGyro[3] = {0};
	int16_t ErrorGyro[3];	
	/*           set offset initial to zero    		*/
	
	memset(MpuOffset,0,12);
	MpuOffset[2] = 8192;   //set offset from the 8192  
	
	HAL_TIM_Base_Stop_IT(&htim3);
	while(k--)//30次静止则判定飞行器处于静止状态
	{
		do
		{
			delay_ms(10);
			MpuGetData();
			for(i=0;i<3;i++)
			{
				ErrorGyro[i] = pMpu[i+3] - LastGyro[i];
				LastGyro[i] = pMpu[i+3];	
			}			
		}while ((ErrorGyro[0] >  MAX_GYRO_QUIET )|| (ErrorGyro[0] < MIN_GYRO_QUIET)//标定静止
					||(ErrorGyro[1] > MAX_GYRO_QUIET )|| (ErrorGyro[1] < MIN_GYRO_QUIET)
					||(ErrorGyro[2] > MAX_GYRO_QUIET )|| (ErrorGyro[2] < MIN_GYRO_QUIET)
						);
	}	

/*           throw first 100  group data and make 256 group average as offset                    */	
	for(i=0;i<356;i++)//水平校准
	{		
		MpuGetData();
		if(100 <= i)//取256组数据进行平均
		{
			uint8_t k;
			for(k=0;k<6;k++)
			{
				buffer[k] += pMpu[k];
			}
		}
	}

	for(i=0;i<6;i++)
	{
		MpuOffset[i] = buffer[i]>>8;
	}
	HAL_TIM_Base_Start_IT(&htim3);
	FLASH_write(MpuOffset,6);//将数据写到FLASH中，一共有6个int16数据
}
/**************************************END OF FILE*************************************/

