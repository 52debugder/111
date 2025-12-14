/*******************************************************************
 *@title LED system
 *@brief flight light
 *@brief 
 *@time  2016.10.19
 *@editor小南&zin
 *飞控爱好QQ群551883670,邮箱759421287@qq.com
 ******************************************************************/
#include "stm32f4xx_hal.h"
#include "LED.h"
#include "ALL_DATA.h"





////右前灯			 
#define fLED1_H()  TIM3->CCR1=1000 //暗
#define fLED1_L()  TIM3->CCR1=500  //亮
#define fLED1_Toggle()  TIM3->CCR1^=(1000^500)//闪烁
////左前灯			 
#define fLED3_H()  TIM3->CCR3=1000 //暗
#define fLED3_L()  TIM3->CCR3=500  //亮
#define fLED3_Toggle()  TIM3->CCR3^=(1000^500)//闪烁
//-------------------------------------------------
////右后灯			 
#define bLED2_H()  TIM3->CCR2=1000 //暗
#define bLED2_L()  TIM3->CCR2=500  //亮
#define bLED2_Toggle()  TIM3->CCR2^=(1000^500)//闪烁
////左后灯			 
#define bLED4_H()  TIM3->CCR4=1000 //暗
#define bLED4_L()  TIM3->CCR4=500  //亮
#define bLED4_Toggle()  TIM3->CCR4^=(1000^500)//闪烁
//-------------------------------------------------

//-------------------------------------------------
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */
sLED LED = {300,AllFlashLight};  //LED initial statue is off;
                             //default 300ms flash the status

/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	

/*
pilot
n. 飞行员；领航员；（船只的）领航员；（电视）试播节目；常燃小火；航海手册；<非正式>职业骑师；<古>向导；（火车头前端的）排障器
adj. 试点的；（电信）控制引导信号
v. 驾驶（飞行器）；领航；试验，试行；使（新法律）顺利通过；<文>带领，指引
*/

void PilotLED() //flash 300MS interval
{
	static uint32_t LastTime = 0;

	if(SysTick_count - LastTime < LED.FlashTime)
	{
		return;
	}
	else
		LastTime = SysTick_count;
	switch(LED.status)
	{
		case AlwaysOff:      //常暗   
			fLED1_H();
			fLED3_H();
			bLED2_H();
			bLED4_H();
			break;
		case AllFlashLight:				  //全部同时闪烁
			//fLED_Toggle();			
			fLED1_Toggle();		
			fLED3_Toggle();	
			bLED2_Toggle();		
			bLED4_Toggle();			
			break;
		case AlwaysOn:  //常亮
			fLED1_L();
			fLED3_L();
			bLED2_L();
			bLED4_L();
		//	fLED_H();
			break;		
		default:
			LED.status = AlwaysOff;
			break;
	}
}

/**************************END OF FILE*********************************/



