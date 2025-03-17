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
extern DMA_HandleTypeDef hdma_spi2_tx;

extern FIL file;

uint8_t audioname[30];
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

uint16_t I2S_Buf0[BUFFER_SIZE] = { 0 };
uint16_t I2S_Buf1[BUFFER_SIZE] = { 0 };
uint8_t *Delta = NULL;

static uint32_t DataLength = 0;
static uint8_t *DataAddress = NULL;

uint32_t bw;
int i;

static void DMAEx_XferCpltCallback(struct __DMA_HandleTypeDef *hdma);
static void DMAEx_XferM1CpltCallback(struct __DMA_HandleTypeDef *hdma);
static void DMAEx_XferErrorCallback(struct __DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_I2S_Transmit_DMAEx(I2S_HandleTypeDef *hi2s,
		uint16_t *FirstBuffer, uint16_t *SecondBuffer, uint16_t Size);

HAL_StatusTypeDef WM8978_Register_Wirter(uint8_t reg_addr, uint16_t data) {
	uint8_t pData[10] = { 0 };

	pData[0] = (reg_addr << 1) | ((data >> 8) & 0x01);
	pData[1] = data & 0xFF;
	return HAL_I2C_Master_Transmit(&USEI2C, WM8978_WIRTE_ADDRESS, pData, 2,
			1000);
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

void Audio_Player_Init(void) {

//	WM8978_Register_Wirter(0, 0);
//	WM8978_Register_Wirter(1, 0x0F);
//	WM8978_Register_Wirter(3, 0x7F);
//	WM8978_Register_Wirter(4, 0x10);
//	WM8978_Register_Wirter(6, 0);
//	WM8978_Register_Wirter(10, 0x08);
//	WM8978_Register_Wirter(43, 0x10);
//	WM8978_Register_Wirter(54, 50);
//	WM8978_Register_Wirter(55, 50 | (1 << 8));



	WM8978_Register_Wirter(0, 0);
	WM8978_Register_Wirter(1, 0x0F);
	WM8978_Register_Wirter(2, 0x180);	// ģ��Ŵ���ʹ�ܣ�?? ʹ��������뻺����??
	WM8978_Register_Wirter(3, 0x7F);
	WM8978_Register_Wirter(4, 0x10);
	WM8978_Register_Wirter(6, 0);
	WM8978_Register_Wirter(10, 0x08);
	WM8978_Register_Wirter(43, 0x10);
	WM8978_Register_Wirter(52,40);		// 设置LOUT2左声道音�????
	WM8978_Register_Wirter(53,40|(1<<8));
	WM8978_Register_Wirter(54, 50);
	WM8978_Register_Wirter(55, 50 | (1 << 8));

}

void Audio_Set_Volume(int num) {
	WM8978_Register_Wirter(0, 0);
	WM8978_Register_Wirter(1, 0x0F);
	WM8978_Register_Wirter(2, 0x180);	// ģ��Ŵ���ʹ�ܣ�?? ʹ��������뻺����??
	WM8978_Register_Wirter(3, 0x7F);
	WM8978_Register_Wirter(4, 0x10);
	WM8978_Register_Wirter(6, 0);
	WM8978_Register_Wirter(10, 0x08);
	WM8978_Register_Wirter(43, 0x10);
	WM8978_Register_Wirter(52, num);		// 设置LOUT2左声道音�????
	WM8978_Register_Wirter(53, num | (1 << 8));
	WM8978_Register_Wirter(54, num);
	WM8978_Register_Wirter(55, num | (1 << 8));
}

void Audio_Player_Start() {
	WAV_FileRead2((uint8_t*) I2S_Buf0, sizeof(I2S_Buf0));
	WAV_FileRead2((uint8_t*) I2S_Buf1, sizeof(I2S_Buf1));
	HAL_I2S_Transmit_DMAEx(&USEI2S, I2S_Buf0, I2S_Buf1, BUFFER_SIZE);
}

void Audio_Player_Start2(void) {
	WAV_FileInit();
	WAV_FileRead((uint8_t*) I2S_Buf0, sizeof(I2S_Buf0));
	WAV_FileRead((uint8_t*) I2S_Buf1, sizeof(I2S_Buf1));
	HAL_I2S_Transmit_DMAEx(&USEI2S, I2S_Buf0, I2S_Buf1, BUFFER_SIZE);
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

