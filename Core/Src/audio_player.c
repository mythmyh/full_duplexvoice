/*
 * audio_player.c
 *
 *  Created on: Jun 9, 2020
 *      Author: admin
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "datas.h"

#include "fatfs.h"
//#include "integer.h"
#include "audio_player.h"
#define USEI2C hi2c2
#define USEI2S hi2s3
//#include "usb_host.h"
#define	BUFFER_SIZE					1024
#define	WM8978_ADDRESS				0x1A
#define	WM8978_WIRTE_ADDRESS		(WM8978_ADDRESS << 1 | 0)
extern I2C_HandleTypeDef USEI2C;
extern I2S_HandleTypeDef USEI2S;

extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s2;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern FIL file;
extern FIL file2;
uint8_t audioname[30];
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

uint16_t I2S_Buf0[BUFFER_SIZE] = { 0 };
uint16_t I2S_Buf1[BUFFER_SIZE] = { 0 };

uint16_t I2S_Buf2[BUFFER_SIZE] = { 0 };
uint16_t I2S_Buf3[BUFFER_SIZE] = { 0 };

uint8_t *Delta = NULL;

static uint32_t DataLength = 0;
static uint8_t *DataAddress = NULL;
FRESULT res2;
uint32_t bw;
uint32_t bw2;
int i;

static void DMAEx_XferCpltCallback(struct __DMA_HandleTypeDef *hdma);
static void DMAEx_XferM1CpltCallback(struct __DMA_HandleTypeDef *hdma);
static void DMAEx_XferErrorCallback(struct __DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_I2S_Transmit_DMAEx(I2S_HandleTypeDef *hi2s,
		uint16_t *FirstBuffer, uint16_t *SecondBuffer, uint16_t Size);

//HAL_StatusTypeDef WM8978_Register_Wirter(uint8_t reg_addr, uint16_t data) {
//	uint8_t pData[10] = { 0 };
//
//	pData[0] = (reg_addr << 1) | ((data >> 8) & 0x01);
//	pData[1] = data & 0xFF;
//	return HAL_I2C_Master_Transmit(&USEI2C, WM8978_WIRTE_ADDRESS, pData, 2,
//			1000);
//}

HAL_StatusTypeDef WM8978_Register_Wirter(I2C_HandleTypeDef *hi2c,uint8_t reg_addr, uint16_t data)
{
	uint8_t pData[10] =	{ 0 };

	pData[0] = (reg_addr << 1) | ((data >> 8) & 0x01);
	pData[1] = data & 0xFF;
	//WM8978_REGVAL_TBL[reg_addr]=data;	//卤拢麓忙录脛麓忙脝梅脰碌碌陆卤戮碌脴

	return HAL_I2C_Master_Transmit(hi2c,WM8978_WIRTE_ADDRESS, pData, 2, 1000);
}




void WAV_FileInit(void) {
	DataLength = sizeof(data) - 0x2c;
	DataAddress = (uint8_t*) (data + 0x2c);
}
uint32_t WAV_FileRead2(uint8_t *buf, uint32_t size) {
	bw = 0;
	f_read(&file, buf, size, (void*)&bw); //16bit音频,直接读取数据
	//printf("aaaa %d\n",bw);

	if (bw < BUFFER_SIZE) //不够数据了,补充0
	{
		for (i = bw; i < BUFFER_SIZE - bw; i++)
			buf[i] = 0;

		f_close(&file);

		return 0;

	}
	return 1;
}


uint32_t WAV_FileRead3(uint8_t *buf, uint32_t size) {
	bw2 = 0;
	res2=f_read(&file2, buf, size, (void*)&bw2); //16bit音频,直接读取数据
	printf("aaaa %d %d\n",res2,bw2);

	if (bw2 < BUFFER_SIZE) //不够数据了,补充0
	{
		for (i = bw; i < BUFFER_SIZE - bw2; i++)
			buf[i] = 0;

		f_close(&file2);

		return 0;

	}
	return 1;
}

uint32_t WAV_FileRead(uint8_t *buf, uint32_t size) {

	uint32_t Playing_End = 0;

	if (DataLength >= size) {
		memcpy(buf, DataAddress, size);
		DataLength -= size;
		DataAddress += size;
		Playing_End = 1;
	} else {
		memcpy(buf, DataAddress, DataLength);
		Playing_End = 0;
	}

	return Playing_End;
}



static void DMAEx_XferCpltCallback2(struct __DMA_HandleTypeDef *hdma) {
	//if (DMA1_Stream3->CR & (1 << 19)) {
	//if(DMA1_Stream4->CR&(1<<19)){
	//printf("aaaa\n");
	if (WAV_FileRead3((uint8_t*) I2S_Buf2, sizeof(I2S_Buf2)) == 0) {
		Audio_Player_Stop();
	}

	//}

}

static void DMAEx_XferM1CpltCallback2(struct __DMA_HandleTypeDef *hdma) {

	if (WAV_FileRead3((uint8_t*) I2S_Buf3, sizeof(I2S_Buf3)) == 0) {
		Audio_Player_Stop();
	}

}




HAL_StatusTypeDef HAL_I2S_Transmit_DMAEx(I2S_HandleTypeDef *hi2s,
		uint16_t *FirstBuffer, uint16_t *SecondBuffer, uint16_t Size) {
	uint32_t tmpreg_cfgr;
	if ((FirstBuffer == NULL) || (SecondBuffer == NULL) || (Size == 0U)) {
		return HAL_ERROR;
	}

	/* Process Locked */
	__HAL_LOCK(hi2s);

	if (hi2s->State != HAL_I2S_STATE_READY) {
		__HAL_UNLOCK(hi2s);
		return HAL_BUSY;
	}

	/* Set state and reset error code */
	hi2s->State = HAL_I2S_STATE_BUSY_TX;
	hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
	hi2s->pTxBuffPtr = FirstBuffer;

	tmpreg_cfgr = hi2s->Instance->I2SCFGR
			& (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN);

	if ((tmpreg_cfgr == I2S_DATAFORMAT_24B)
			|| (tmpreg_cfgr == I2S_DATAFORMAT_32B)) {
		hi2s->TxXferSize = (Size << 1U);
		hi2s->TxXferCount = (Size << 1U);
	} else {
		hi2s->TxXferSize = Size;
		hi2s->TxXferCount = Size;
	}

	/* Set the I2S Tx DMA Half transfer complete callback */
	hi2s->hdmatx->XferHalfCpltCallback = NULL;
	hi2s->hdmatx->XferM1HalfCpltCallback = NULL;

	/* Set the I2S Tx DMA transfer complete callback */
	hi2s->hdmatx->XferCpltCallback = DMAEx_XferCpltCallback;
	hi2s->hdmatx->XferM1CpltCallback = DMAEx_XferM1CpltCallback;

	/* Set the DMA error callback */
	hi2s->hdmatx->XferErrorCallback = DMAEx_XferErrorCallback;

	/* Set the DMA abort callback */
	hi2s->hdmatx->XferAbortCallback = NULL;

	/* Enable the Tx DMA Stream/Channel */
	if (HAL_OK
			!= HAL_DMAEx_MultiBufferStart_IT(hi2s->hdmatx,
					(uint32_t) FirstBuffer, (uint32_t) &hi2s->Instance->DR,
					(uint32_t) SecondBuffer, hi2s->TxXferSize)) {
		/* Update SPI error code */
		SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_DMA);
		hi2s->State = HAL_I2S_STATE_READY;

		__HAL_UNLOCK(hi2s);
		return HAL_ERROR;
	}

	/* Check if the I2S is already enabled */
	if (HAL_IS_BIT_CLR(hi2s->Instance->I2SCFGR, SPI_I2SCFGR_I2SE)) {
		/* Enable I2S peripheral */
		__HAL_I2S_ENABLE(hi2s);
	}

	/* Check if the I2S Tx request is already enabled */
	if (HAL_IS_BIT_CLR(hi2s->Instance->CR2, SPI_CR2_TXDMAEN)) {
		/* Enable Tx DMA Request */
		SET_BIT(hi2s->Instance->CR2, SPI_CR2_TXDMAEN);
	}

	__HAL_UNLOCK(hi2s);
	return HAL_OK;
}

HAL_StatusTypeDef HAL_I2S_Transmit_DMAEx2(I2S_HandleTypeDef *hi2s,
		uint16_t *FirstBuffer, uint16_t *SecondBuffer, uint16_t Size) {
	uint32_t tmpreg_cfgr;
	if ((FirstBuffer == NULL) || (SecondBuffer == NULL) || (Size == 0U)) {
		return HAL_ERROR;
	}

	/* Process Locked */
	__HAL_LOCK(hi2s);

	if (hi2s->State != HAL_I2S_STATE_READY) {
		__HAL_UNLOCK(hi2s);
		return HAL_BUSY;
	}

	/* Set state and reset error code */
	hi2s->State = HAL_I2S_STATE_BUSY_TX;
	hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
	hi2s->pTxBuffPtr = FirstBuffer;

	tmpreg_cfgr = hi2s->Instance->I2SCFGR
			& (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN);

	if ((tmpreg_cfgr == I2S_DATAFORMAT_24B)
			|| (tmpreg_cfgr == I2S_DATAFORMAT_32B)) {
		hi2s->TxXferSize = (Size << 1U);
		hi2s->TxXferCount = (Size << 1U);
	} else {
		hi2s->TxXferSize = Size;
		hi2s->TxXferCount = Size;
	}

	/* Set the I2S Tx DMA Half transfer complete callback */
	hi2s->hdmatx->XferHalfCpltCallback = NULL;
	hi2s->hdmatx->XferM1HalfCpltCallback = NULL;

	/* Set the I2S Tx DMA transfer complete callback */
	hi2s->hdmatx->XferCpltCallback = DMAEx_XferCpltCallback2;
	hi2s->hdmatx->XferM1CpltCallback = DMAEx_XferM1CpltCallback2;

	/* Set the DMA error callback */
	hi2s->hdmatx->XferErrorCallback = DMAEx_XferErrorCallback;

	/* Set the DMA abort callback */
	hi2s->hdmatx->XferAbortCallback = NULL;

	/* Enable the Tx DMA Stream/Channel */
	if (HAL_OK
			!= HAL_DMAEx_MultiBufferStart_IT(hi2s->hdmatx,
					(uint32_t) FirstBuffer, (uint32_t) &hi2s->Instance->DR,
					(uint32_t) SecondBuffer, hi2s->TxXferSize)) {
		/* Update SPI error code */
		SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_DMA);
		hi2s->State = HAL_I2S_STATE_READY;

		__HAL_UNLOCK(hi2s);
		return HAL_ERROR;
	}

	/* Check if the I2S is already enabled */
	if (HAL_IS_BIT_CLR(hi2s->Instance->I2SCFGR, SPI_I2SCFGR_I2SE)) {
		/* Enable I2S peripheral */
		__HAL_I2S_ENABLE(hi2s);
	}

	/* Check if the I2S Tx request is already enabled */
	if (HAL_IS_BIT_CLR(hi2s->Instance->CR2, SPI_CR2_TXDMAEN)) {
		/* Enable Tx DMA Request */
		SET_BIT(hi2s->Instance->CR2, SPI_CR2_TXDMAEN);
	}

	__HAL_UNLOCK(hi2s);
	return HAL_OK;
}








static void DMAEx_XferCpltCallback(struct __DMA_HandleTypeDef *hdma) {
	//if (DMA1_Stream3->CR & (1 << 19)) {
	//if(DMA1_Stream4->CR&(1<<19)){
	if (WAV_FileRead2((uint8_t*) I2S_Buf0, sizeof(I2S_Buf0)) == 0) {
		Audio_Player_Stop();
	}

	//}

}

static void DMAEx_XferM1CpltCallback(struct __DMA_HandleTypeDef *hdma) {

	if (WAV_FileRead2((uint8_t*) I2S_Buf1, sizeof(I2S_Buf1)) == 0) {
		Audio_Player_Stop();
	}

}

static void DMAEx_XferErrorCallback(struct __DMA_HandleTypeDef *hdma) {

}

void Audio_Player_Init(I2C_HandleTypeDef*hi2c) {

	WM8978_Register_Wirter(hi2c,0, 0);
		WM8978_Register_Wirter(hi2c,1,0X1B);	//R1,MICEN脡猫脰脙脦陋1(MIC脢鹿脛脺),BIASEN脡猫脰脙脦陋1(脛拢脛芒脝梅鹿陇脳梅),VMIDSEL[1:0]脡猫脰脙脦陋:11(5K)
		WM8978_Register_Wirter(hi2c,2,0X1B0);	//R2,ROUT1,LOUT1脢盲鲁枚脢鹿脛脺(露煤禄煤驴脡脪脭鹿陇脳梅),BOOSTENR,BOOSTENL脢鹿脛脺
		WM8978_Register_Wirter(hi2c,3,0X6C);	//R3,LOUT2,ROUT2脢盲鲁枚脢鹿脛脺(脌庐掳脠鹿陇脳梅),RMIX,LMIX脢鹿脛脺
		WM8978_Register_Wirter(hi2c,3, 0x7F);
		WM8978_Register_Wirter(hi2c,4, 0x10);
		WM8978_Register_Wirter(hi2c,6,0);		//R6,MCLK脫脡脥芒虏驴脤谩鹿漏
		WM8978_Register_Wirter(hi2c,43,1<<4);	//R43,INVROUT2路麓脧貌,脟媒露炉脌庐掳脠
		WM8978_Register_Wirter(hi2c,47,1<<8);	//R47脡猫脰脙,PGABOOSTL,脳贸脥篓碌脌MIC禄帽碌脙20卤露脭枚脪忙
		WM8978_Register_Wirter(hi2c,48,1<<8);	//R48脡猫脰脙,PGABOOSTR,脫脪脥篓碌脌MIC禄帽碌脙20卤露脭枚脪忙
		WM8978_Register_Wirter(hi2c,49,1<<1);	//R49,TSDEN,驴陋脝么鹿媒脠脠卤拢禄陇
		WM8978_Register_Wirter(hi2c,10,1<<3);	//R10,SOFTMUTE鹿脴卤脮,128x虏脡脩霉,脳卯录脩SNR
		WM8978_Register_Wirter(hi2c,14,1<<3);	//R14,AD

}



void Audio_Player_Start() {
	WAV_FileRead3((uint8_t*) I2S_Buf2, sizeof(I2S_Buf0));
	WAV_FileRead3((uint8_t*) I2S_Buf3, sizeof(I2S_Buf1));
	HAL_I2S_Transmit_DMAEx(&USEI2S, I2S_Buf0, I2S_Buf1, BUFFER_SIZE);

	HAL_I2S_Transmit_DMAEx2(&hi2s2, I2S_Buf2, I2S_Buf3, BUFFER_SIZE);

}



void Audio_Player_Pause(void) {
	HAL_I2S_DMAPause(&USEI2S);
}

void Audio_Player_Resume(void) {
	HAL_I2S_DMAResume(&USEI2S);
}

void Audio_Player_Stop(void) {
	WAV_FileInit();
	HAL_I2S_DMAStop(&USEI2S);
}

