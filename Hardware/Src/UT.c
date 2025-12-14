#include "stm32f4xx_hal.h"
#include "Delay.h"

extern float Distance;

#define UT_PORT GPIOB
#define UT_TIME TIM4
#define TRIG GPIO_Pin_7
#define ECHO GPIO_Pin_6

void UT_Init(void){
//	TIM_ICInitTypeDef UT_IC;
//	TIM_TimeBaseInitTypeDef UT_TimeBase;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

//	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	GPIO_InitStructure.GPIO_Pin = ECHO;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(UT_PORT, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Pin = TRIG;
//	GPIO_Init(UT_PORT, &GPIO_InitStructure);
//	
//	TIM_InternalClockConfig(TIM4);
//	
//	GPIO_SetBits(UT_PORT, TRIG);
//	
//	
//	UT_TimeBase.TIM_ClockDivision = TIM_CKD_DIV1;
//	UT_TimeBase.TIM_CounterMode = TIM_CounterMode_Up;
//	UT_TimeBase.TIM_Period = 65536 - 1;
//	UT_TimeBase.TIM_Prescaler = 72 - 1;
//	UT_TimeBase.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(UT_TIME, &UT_TimeBase);
//	

//	UT_IC.TIM_Channel = TIM_Channel_1;
//	UT_IC.TIM_ICFilter = 0xF;
//	UT_IC.TIM_ICPolarity = TIM_ICPolarity_Rising;
//	UT_IC.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//	UT_IC.TIM_ICSelection = TIM_ICSelection_DirectTI;
//	TIM_ICInit(TIM4, &UT_IC);
//	UT_IC.TIM_Channel = TIM_Channel_2;
//	UT_IC.TIM_ICSelection = TIM_ICSelection_IndirectTI;
//	UT_IC.TIM_ICPolarity = TIM_ICPolarity_Falling;
//	TIM_ICInit(TIM4, &UT_IC);
//	
//	TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);
//	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
//	
//	
//	TIM_Cmd(TIM4, ENABLE);
}

void UT_GetDistance()
{
//	GPIO_ResetBits(UT_PORT, TRIG);
//	delay_us(20);
//	GPIO_SetBits(UT_PORT, TRIG);
}
