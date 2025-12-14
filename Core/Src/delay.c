#include "stm32f4xx_hal.h"
#include "delay.h"
#include "ALL_DATA.h"
static volatile uint32_t usTicks = 0;

/*必须在时钟初始化之后再进行调用，否则会获取到默认的低频率，导致usTicks错误*/
void cycleCounterInit(void)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t flashLatency;
    HAL_RCC_GetClockConfig(&clocks, &flashLatency);
    usTicks = clocks.SYSCLKSource / 1000000;
}


uint32_t GetSysTime_us(void) 
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = SysTick_count;
        cycle_cnt = SysTick->VAL;
    	} while (ms != SysTick_count);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

//    毫秒级延时函数	 
void delay_ms(uint16_t nms)
{
	uint32_t t0=GetSysTime_us();
	while(GetSysTime_us() - t0 < nms * 1000);	  	  
}

void delay_us(unsigned int i)
 {  
	char x=0;   
    while( i--)
    {	
       for(x=1;x>0;x--);
    }
 }		  
