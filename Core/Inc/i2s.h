#ifndef __I2S_H
#define __I2S_H
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//I2S 驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/5/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
//******************************************************************************** 
//V1.1 20141220  
//修正I2S2_SampleRate_Set函数ODD位设置的bug
////////////////////////////////////////////////////////////////////////////////// 	
#define I2S_RX_DMA_BUF_SIZE    	4096		//¶¨ÒåRX DMA Êý×é´óÐ¡
 
extern void (*i2s_tx_callback)(void);		//IIS TX回调函数指针
extern void (*i2s_rx_callback)(void);		//IIS RX回调函数指针

void I2S2_Init(uint32_t I2S_Standard,uint32_t I2S_Mode,uint32_t I2S_Clock_Polarity,uint32_t I2S_DataFormat);
//void I2S2ext_Init(uint32_t I2S_Standard,uint32_t I2S_Mode,uint32_t I2S_Clock_Polarity,uint32_t I2S_DataFormat);
void I2S2ext_Init(uint8_t std,uint8_t mode,uint8_t cpol,uint8_t datalen);
uint8_t I2S2_SampleRate_Set(uint32_t samplerate);
void I2S2_TX_DMA_Init(uint8_t* buf0,uint8_t *buf1,uint16_t num);
void I2S2ext_RX_DMA_Init(uint8_t* buf0,uint8_t *buf1,uint16_t num);
void I2S_Play_Start(void); 
void I2S_Rec_Start(void);
void I2S_Play_Stop(void);
void I2S_Rec_Stop(void);
#endif





















