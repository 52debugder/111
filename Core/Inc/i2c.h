/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c2;

extern I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN Private defines */
#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#define TRUE  0
#define FALSE -1

  #define IIC_RCC       RCC_APB2Periph_GPIOB
  #define IIC_GPIO      GPIOB
  #define SCL_PIN       GPIO_Pin_8
  #define SDA_PIN       GPIO_Pin_9

#define SCL_H         GPIOB->BSRR = GPIO_Pin_8 /* GPIO_SetBits(GPIOB , GPIO_Pin_10)   */
#define SCL_L         GPIOB->BRR  = GPIO_Pin_8 /* GPIO_ResetBits(GPIOB , GPIO_Pin_10) */

#define SDA_H         GPIOB->BSRR = GPIO_Pin_9 /* GPIO_SetBits(GPIOB , GPIO_Pin_11)   */
#define SDA_L         GPIOB->BRR  = GPIO_Pin_9 /* GPIO_ResetBits(GPIOB , GPIO_Pin_11) */

#define SCL_read      GPIOB->IDR  & GPIO_Pin_8 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_10) */
#define SDA_read      GPIOB->IDR  & GPIO_Pin_9 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_11) */


//0表示写
#define	I2C_Direction_Trans   0
//１表示读
#define	I2C_Direction_Rec 
/* USER CODE END Private defines */

void MX_I2C2_Init(void);
void MX_I2C3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

