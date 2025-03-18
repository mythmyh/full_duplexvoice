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
#define	BUFFER_SIZE					512
#define	WM8978_ADDRESS				0x1A
#define	WM8978_WIRTE_ADDRESS		(WM8978_ADDRESS << 1 | 0)
extern I2C_HandleTypeDef USEI2C;
extern I2S_HandleTypeDef USEI2S;

extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s2;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern FIL file;
extern FIL file2;
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


static uint16_t WM8978_REGVAL_TBL[58]=
{
	0X0000,0X0000,0X0000,0X0000,0X0050,0X0000,0X0140,0X0000,
	0X0000,0X0000,0X0000,0X00FF,0X00FF,0X0000,0X0100,0X00FF,
	0X00FF,0X0000,0X012C,0X002C,0X002C,0X002C,0X002C,0X0000,
	0X0032,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
	0X0038,0X000B,0X0032,0X0000,0X0008,0X000C,0X0093,0X00E9,
	0X0000,0X0000,0X0000,0X0000,0X0003,0X0010,0X0010,0X0100,
	0X0100,0X0002,0X0001,0X0001,0X0039,0X0039,0X0039,0X0039,
	0X0001,0X0001
};

uint16_t WM8978_Read_Reg(uint8_t reg)
{
	return WM8978_REGVAL_TBL[reg];
}
HAL_StatusTypeDef WM8978_Register_Wirter2(uint8_t reg_addr, uint16_t data)
{
	uint8_t pData[10] =	{ 0 };

	pData[0] = (reg_addr << 1) | ((data >> 8) & 0x01);
	pData[1] = data & 0xFF;
	WM8978_REGVAL_TBL[reg_addr]=data;	//保存寄存器值到本地

	return HAL_I2C_Master_Transmit(&hi2c1, WM8978_WIRTE_ADDRESS, pData, 2, 1000);
}
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
void WM8978_Output_Cfg(uint8_t dacen,uint8_t bpsen)
{
	uint16_t regval=0;
	if(dacen)regval|=1<<0;	//DAC脢盲鲁枚脢鹿脛脺
	if(bpsen)
	{
		regval|=1<<1;		//BYPASS脢鹿脛脺
		regval|=5<<2;		//0dB脭枚脪忙
	}
	WM8978_Register_Wirter2(50,regval);//R50脡猫脰脙
	WM8978_Register_Wirter2(51,regval);//R51脡猫脰脙
}

void WM8978_AUX_Gain(uint8_t gain)
{
	uint16_t regval;
	gain&=0X07;
	regval=WM8978_Read_Reg(47);//读取R47
	regval&=~(7<<0);			//清除原来的设置
	WM8978_Register_Wirter2(47,regval|gain<<0);//设置R47
	regval=WM8978_Read_Reg(48);	//读取R48
	regval&=~(7<<0);			//清除原来的设置
	WM8978_Register_Wirter2(48,regval|gain<<0);//设置R48
}



void WAV_FileInit(void) {
	DataLength = sizeof(data) - 0x2c;
	DataAddress = (uint8_t*) (data + 0x2c);
}
uint32_t WAV_FileRead2(uint8_t *buf, uint32_t size) {
	bw = 0;
	f_read(&file, buf, size, (void*)&bw); //16bit音频,直接读取数据
	printf("aaaa %d\n",bw);

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


	//HAL_I2S_Transmit_DMAEx2(&hi2s2, I2S_Buf2, I2S_Buf3, BUFFER_SIZE);
}


void WM8978_HPvol_Set(uint8_t voll,uint8_t volr)
{
	voll&=0X3F;
	volr&=0X3F;//脧脼露篓路露脦搂
	if(voll==0)voll|=1<<6;//脪么脕驴脦陋0脢卤,脰卤陆脫mute
	if(volr==0)volr|=1<<6;//脪么脕驴脦陋0脢卤,脰卤陆脫mute
	WM8978_Register_Wirter2(52,voll);			//R52,露煤禄煤脳贸脡霉碌脌脪么脕驴脡猫脰脙
	WM8978_Register_Wirter2(53,volr|(1<<8));	//R53,露煤禄煤脫脪脡霉碌脌脪么脕驴脡猫脰脙,脥卢虏陆赂眉脨脗(HPVU=1)
}



void WM8978_SPKvol_Set(uint8_t volx)
{
	volx&=0X3F;//脧脼露篓路露脦搂
	if(volx==0)volx|=1<<6;//脪么脕驴脦陋0脢卤,脰卤陆脫mute
	WM8978_Register_Wirter2(54,volx);			//R54,脌庐掳脠脳贸脡霉碌脌脪么脕驴脡猫脰脙
	WM8978_Register_Wirter2(55,volx|(1<<8));	//R55,脌庐掳脠脫脪脡霉碌脌脪么脕驴脡猫脰脙,脥卢虏陆赂眉脨脗(SPKVU=1)
}

void WM8978_ADDA_Cfg(uint8_t dacen,uint8_t adcen)
{
	uint16_t regval;
	regval=WM8978_Read_Reg(3);	//读取R3
	if(dacen)regval|=3<<0;		//R3最低2个位设置为1,开启DACR&DACL
	else regval&=~(3<<0);		//R3最低2个位清零,关闭DACR&DACL.
	WM8978_Register_Wirter2(3,regval);	//设置R3
	regval=WM8978_Read_Reg(2);	//读取R2
	if(adcen)regval|=3<<0;		//R2最低2个位设置为1,开启ADCR&ADCL
	else regval&=~(3<<0);		//R2最低2个位清零,关闭ADCR&ADCL.
	WM8978_Register_Wirter2(2,regval);	//设置R2
}

void WM8978_LINEIN_Gain(uint8_t gain)
{
	uint16_t regval;
	gain&=0X07;
	regval=WM8978_Read_Reg(47);	//露脕脠隆R47
	regval&=~(7<<4);			//脟氓鲁媒脭颅脌麓碌脛脡猫脰脙
	WM8978_Register_Wirter2(47,regval|gain<<4);//脡猫脰脙R47
	regval=WM8978_Read_Reg(48);	//露脕脠隆R48
	regval&=~(7<<4);			//脟氓鲁媒脭颅脌麓碌脛脡猫脰脙
	WM8978_Register_Wirter2(48,regval|gain<<4);//脡猫脰脙R48
}

void WM8978_Input_Cfg(uint8_t micen,uint8_t lineinen,uint8_t auxen)
{
	uint16_t regval;
	regval=WM8978_Read_Reg(2);	//读取R2
	if(micen)regval|=3<<2;		//开启INPPGAENR,INPPGAENL(MIC的PGA放大)
	else regval&=~(3<<2);		//关闭INPPGAENR,INPPGAENL.
 	WM8978_Register_Wirter2(2,regval);	//设置R2

	regval=WM8978_Read_Reg(44);	//读取R44
	if(micen)regval|=3<<4|3<<0;	//开启LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	else regval&=~(3<<4|3<<0);	//关闭LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	WM8978_Register_Wirter2(44,regval);//设置R44

	if(lineinen)WM8978_LINEIN_Gain(5);//LINE IN 0dB增益
	else WM8978_LINEIN_Gain(0);		//关闭LINE IN
	if(auxen)WM8978_AUX_Gain(7);//AUX 6dB增益
	else WM8978_AUX_Gain(0);	//关闭AUX输入
}

void WM8978_MIC_Gain(uint8_t gain)
{
	gain&=0X3F;
	WM8978_Register_Wirter2(45,gain);		//R45,脳贸脥篓碌脌PGA脡猫脰脙
	WM8978_Register_Wirter2(46,gain|1<<8);	//R46,脫脪脥篓碌脌PGA脡猫脰脙
}
void WM8978_I2S_Cfg(uint8_t fmt,uint8_t len)
{
	fmt&=0X03;
	len&=0X03;//限定范围
	WM8978_Register_Wirter2(4,(fmt<<3)|(len<<5));	//R4,WM8978工作模式设置
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

