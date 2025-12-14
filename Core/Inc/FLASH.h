#ifndef _FLASH_H_
#define _FLASH_H_
#include "stm32f4xx_hal.h"

#define FLASH_Page_Size    (0x00000400) //FLASH页的大小1K
#define FLASH_Start_Addr   (0x08007c00) //要编写的起始地址
#define FLASH_End_Addr     (0x08007fff) //要编写的结束地址
#define DATA_32            (0x12345678) //要编写的数据

extern void FLASH_read(int16_t *data,uint8_t len);
extern void FLASH_write(int16_t *data,uint8_t len);	
void FLASH_erase_page(uint32_t page_address);
void FLASH_erase_pages(uint32_t start_page_addr, uint32_t num_pages);

#endif
