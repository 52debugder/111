#include "NRF24L01.h"

uint8_t NRF24L01_RXAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define NRF24L01_RX_PACKET_WIDTH    4   // 接收数据包宽度
uint8_t NRF24L01_RxPacket[NRF24L01_RX_PACKET_WIDTH];

uint8_t NRF24L01_TXAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
#define NRF24L01_TX_PACKET_WIDTH    4   // 发送数据包宽度
uint8_t NRF24L01_TxPacket[NRF24L01_TX_PACKET_WIDTH];

uint8_t state;

void NRF24L01_Init(void)
{
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
    
  NRF24L01_GPIO_Init();
	
	NRF24L01_WriteReg(NRF24L01_CONFIG, 0x08);		
	NRF24L01_WriteReg(NRF24L01_EN_AA, 0x3F);		
	NRF24L01_WriteReg(NRF24L01_EN_RXADDR, 0x01);	
	NRF24L01_WriteReg(NRF24L01_SETUP_AW, 0x03);		
	NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0x03);	
	NRF24L01_WriteReg(NRF24L01_RF_CH, 0x40);		
	NRF24L01_WriteReg(NRF24L01_RF_SETUP, 0x0E);

    NRF24L01_WriteReg(NRF24L01_RX_PW_P0, NRF24L01_RX_PACKET_WIDTH);
    NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RXAddress, 5);
    NRF24L01_TXFLUSH(); // 清空TX_FIFO
	NRF24L01_RXFLUSH(); // 清空RX_FIFO
	NRF24L01_WriteReg(NRF24L01_STATUS, 0x70);
	NRF24L01_RX(); // 默认进入接收模式
    /*
    TIM_InternalClockConfig(TIM2);
    
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 10000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM2, ENABLE);
    
    */
}

/*
void TIM2_IRQHandler()
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        
        OLED_ShowNum(3, 3, 111, 3);
        
        state = NRF24L01_Send();
    }
}
*/

uint8_t NRF24L01_Send(void)
{
    uint32_t Time_Out = 0;
    uint8_t SendFlag = 0;
    uint8_t Status = 0;
    
    NRF24L01_WriteRegs(NRF24L01_TX_ADDR, NRF24L01_TXAddress, 5);
    NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_TXAddress, 5);
    NRF24L01_WriteTXPayLoad(NRF24L01_TxPacket, NRF24L01_TX_PACKET_WIDTH);
    NRF24L01_Tr();
    
    
    Time_Out = 400;
    
    while(1)
    {
        Time_Out--;
        if(Time_Out == 0)
        {
            SendFlag = 4;
//            OLED_ShowNum(3, 3, NRF24L01_ReadReg(NRF24L01_CONFIG), 2);
            NRF24L01_Init();
            
            break;
        }
        
        
        
        Status = NRF24L01_ReadStatus();
        if(Status == 0x30) // 状态寄存器4(MAX_RT)和状态寄存器5(TX_DS)都为1
        {
            SendFlag = 3;
            NRF24L01_Init();
              break;
        }
        else if(Status == 0x10) // 状态寄存器4(MAX_RT)为1
        {
            SendFlag = 2;
            NRF24L01_Init();
              break;
        }
        else if(Status == 0x20) // 状态寄存器5(TX_DS)为1，发送正确
        {
            SendFlag = 1;
              break;
        }
    }
    
    NRF24L01_WriteReg(NRF24L01_STATUS, 0x30);
    NRF24L01_TXFLUSH();
    NRF24L01_WriteRegs(NRF24L01_RX_ADDR_P0, NRF24L01_RXAddress, 5); // 恢复接收通道原来的地址
    NRF24L01_RX(); // 返回接收模式
    
    return SendFlag;
}

uint8_t NRF24L01_Receive(void)
{
    uint8_t Status;
    uint8_t Config;
    uint8_t ReceiveFlag;
    
    Status = NRF24L01_ReadStatus();
    Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
    
    if ((Config & 0x02) == 0x00)		
	{
		ReceiveFlag = 3;				
		NRF24L01_Init();				
	}
	else if ((Status & 0x30) == 0x30)	
	{
		ReceiveFlag = 2;				
		NRF24L01_Init();				
	}
	else if ((Status & 0x40) == 0x40)	
    {
        ReceiveFlag = 1;
        
        NRF24L01_ReadRXPayLoad(NRF24L01_RxPacket, NRF24L01_RX_PACKET_WIDTH);
        NRF24L01_WriteReg(NRF24L01_STATUS, 0x40);
        NRF24L01_RXFLUSH();
    }
    else
    {
        ReceiveFlag = 0;
    }
    
    return ReceiveFlag;
}

void NRF24L01_GPIO_Init(void)
{
    NRF24L01_W_CE(0);
    NRF24L01_W_SCN(1);
    NRF24L01_W_SCK(0);
    NRF24L01_W_MOSI(0);
}

void NRF24L01_W_CE(uint8_t bit)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, (GPIO_PinState)bit);
}

void NRF24L01_W_SCN(uint8_t bit)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, (GPIO_PinState)bit);
}

void NRF24L01_W_SCK(uint8_t bit)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (GPIO_PinState)bit);
}

void NRF24L01_W_MOSI(uint8_t bit)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (GPIO_PinState)bit);
}

uint8_t NRF24L01_R_MISO(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
}

//交换一个字节
uint8_t NRF24L01_SPI_SwapDate(uint8_t Date)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        if(Date & 0x80) NRF24L01_W_MOSI(1);
        else NRF24L01_W_MOSI(0);
        
        Date <<= 1;
        
        NRF24L01_W_SCK(1);
        if(NRF24L01_R_MISO()) Date |= 0x01;
        NRF24L01_W_SCK(0);
    }
    
    return Date;
}

//读取寄存器一个字节
uint8_t NRF24L01_ReadReg(uint8_t RegAddress)
{
    uint8_t Date;
    
    NRF24L01_W_SCN(0); // 通信开始
    
    NRF24L01_SPI_SwapDate(NRF24L01_R_REGISTER | RegAddress); // 发送读寄存器指令 
    Date = NRF24L01_SPI_SwapDate(NRF24L01_NOP); // 发送垃圾数据得到寄存器数据
    
    NRF24L01_W_SCN(1); // 通信结束
    
    return Date;
}

//读取寄存器多个字节
void NRF24L01_ReadRegs(uint8_t RegAddress, uint8_t *DateArray, uint8_t Count)
{
    NRF24L01_W_SCN(0); // 通信开始
    NRF24L01_SPI_SwapDate(NRF24L01_R_REGISTER | RegAddress); // 发送读寄存器指令 
    
    for(uint8_t i = 0; i < Count; i++)
    {
        DateArray[i] = NRF24L01_SPI_SwapDate(NRF24L01_NOP); // 发送垃圾数据得到寄存器数据
    }
    
    NRF24L01_W_SCN(1); // 通信结束
}

//写入寄存器一个字节
void NRF24L01_WriteReg(uint8_t RegAddress, uint8_t Date)
{
    NRF24L01_W_SCN(0); // 通信开始
    
    NRF24L01_SPI_SwapDate(NRF24L01_W_REGISTER | RegAddress); // 发送写入寄存器指令 
    Date = NRF24L01_SPI_SwapDate(Date); // 发送垃圾数据得到寄存器数据
    
    NRF24L01_W_SCN(1); // 通信结束
}

//写入寄存器多个字节
void NRF24L01_WriteRegs(uint8_t RegAddress, uint8_t *DateArray, uint8_t Count)
{
    NRF24L01_W_SCN(0); // 通信开始
    NRF24L01_SPI_SwapDate(NRF24L01_W_REGISTER | RegAddress); // 发送写入存器指令 
    
    for(uint8_t i = 0; i < Count; i++)
    {
        NRF24L01_SPI_SwapDate(DateArray[i]); // 发送垃圾数据得到寄存器数据
    }
    
    NRF24L01_W_SCN(1); // 通信结束
}

//读取RX有效载荷
void NRF24L01_ReadRXPayLoad(uint8_t *DateArray, uint8_t Count)
{
    NRF24L01_W_SCN(0); // 通信开始
    
    NRF24L01_SPI_SwapDate(NRF24L01_R_RX_PAYLOAD); // 发送读取存器指令 
    for(uint8_t i = 0; i < Count; i++)
    {
        DateArray[i] = NRF24L01_SPI_SwapDate(NRF24L01_NOP);
    }
    
    NRF24L01_W_SCN(1); // 通信结束
}

//写入TX有效载荷
void NRF24L01_WriteTXPayLoad(uint8_t *DateArray, uint8_t Count)
{
    NRF24L01_W_SCN(0); // 通信开始
    
    NRF24L01_SPI_SwapDate(NRF24L01_W_TX_PAYLOAD); // 发送写入存器指令 
    for(uint8_t i = 0; i < Count; i++)
    {
        NRF24L01_SPI_SwapDate(DateArray[i]);
    }
    
    NRF24L01_W_SCN(1); // 通信结束
}

//清空TX_FIFO
void NRF24L01_TXFLUSH(void)
{
    NRF24L01_W_SCN(0); // 通信开始
    
    NRF24L01_SPI_SwapDate(NRF24L01_FLUSH_TX); // 发送写入存器指令, 清空TX_FIFO
    
    NRF24L01_W_SCN(1); // 通信结束
}

//清空RX_FIFO
void NRF24L01_RXFLUSH(void)
{
    NRF24L01_W_SCN(0); // 通信开始
    
    NRF24L01_SPI_SwapDate(NRF24L01_FLUSH_RX); // 发送写入存器指令, 清空RX_FIFO
    
    NRF24L01_W_SCN(1); // 通信结束
}

// 读取状态寄存器
uint8_t NRF24L01_ReadStatus(void)
{
    NRF24L01_W_SCN(0); // 通信开始
    
    uint8_t status =  NRF24L01_SPI_SwapDate(NRF24L01_NOP); // 
    
    NRF24L01_W_SCN(1); // 通信结束
    
    return status;
}

// 配置掉电模式
void NRF24L01_PowerDown(void)
{
    NRF24L01_W_CE(0); // 退出收发模式
    
    uint8_t Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
    
    if(Config == 0xFF) return;
    Config &= ~0x02; // PWR_UP置0
    
    NRF24L01_WriteReg(NRF24L01_CONFIG, Config); // 写回配置寄存器
}

//配置待机模式
void NRF24L01_StandBy1(void)
{
    NRF24L01_W_CE(0); // 退出收发模式
    
    uint8_t Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
    
    if(Config == 0xFF) return;
    Config |= 0x02; // PWR_UP置1
    
    NRF24L01_WriteReg(NRF24L01_CONFIG, Config); // 写回配置寄存器
}

//配置接收模式
void NRF24L01_RX(void)
{
    NRF24L01_W_CE(0); // 退出收发模式
    
    uint8_t Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
    
    if(Config == 0xFF) return;
    Config |= 0x03; // PWR_UP置1 PRIM_RX置1
    
    NRF24L01_WriteReg(NRF24L01_CONFIG, Config); // 写回配置寄存器
    
    NRF24L01_W_CE(1); // 进入收发模式
}

//配置发送模式
void NRF24L01_Tr(void)
{
    NRF24L01_W_CE(0); // 退出收发模式
    
    uint8_t Config = NRF24L01_ReadReg(NRF24L01_CONFIG);
    
    if(Config == 0xFF) return;
    Config |= 0x02; // PWR_UP置1
    Config &= ~0x01; // PRIM_RX置0
      
    NRF24L01_WriteReg(NRF24L01_CONFIG, Config); // 写回配置寄存器
    
    NRF24L01_W_CE(1); // 进入收发模式
    delay_us(20);
}


