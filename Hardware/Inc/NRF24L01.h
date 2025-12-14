#ifndef __NRF24L01_H_
#define __NRF24L01_H_

#include "stm32f4xx_hal.h"
#include "NRF24L01_Define.h"
#include "delay.h"
#include "OLED.h"

extern uint8_t NRF24L01_RXAddress[];
extern uint8_t NRF24L01_RxPacket[];

extern uint8_t NRF24L01_TXAddress[];
extern uint8_t NRF24L01_TxPacket[];

extern uint8_t state;

/*³õÊ¼»¯*/
void NRF24L01_Init(void);
void NRF24L01_GPIO_Init(void);
uint8_t NRF24L01_Send(void);
uint8_t NRF24L01_Receive(void);

/*SPI¶ÁÐ´*/
void NRF24L01_W_CE(uint8_t bit);
void NRF24L01_W_SCN(uint8_t bit);
void NRF24L01_W_SCK(uint8_t bit);
void NRF24L01_W_MOSI(uint8_t bit);
uint8_t NRF24L01_R_MISO(void);

/*¶ÁÐ´¼Ä´æÆ÷*/
uint8_t NRF24L01_SPI_SwapDate(uint8_t Date);
uint8_t NRF24L01_ReadReg(uint8_t RegAddress);
void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DateArray, uint8_t Count);
void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Date);
void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DateArray, uint8_t Count);

/*¶ÁÐ´ÓÐÐ§ÔØºÉ*/
void NRF24L01_ReadRXPayLoad(uint8_t *DateArray, uint8_t Count);
void NRF24L01_WriteTXPayLoad(uint8_t *DateArray, uint8_t Count);

void NRF24L01_TXFLUSH(void);
void NRF24L01_RXFLUSH(void);
uint8_t NRF24L01_ReadStatus(void);

/*Ä£Ê½Ñ¡Ôñ*/
void NRF24L01_PowerDown(void);
void NRF24L01_StandBy1(void);
void NRF24L01_RX(void);
void NRF24L01_Tr(void);

#endif
