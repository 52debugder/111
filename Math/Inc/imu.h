#ifndef __IMU_H
#define	__IMU_H


#include "ALL_DATA.h"


//extern float GetNormAccz(void);
extern void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt);
extern void imu_rest(void);

float Kalman_Filter(Kalman *KF, float input);
void Kalman_Init(void);


#endif
