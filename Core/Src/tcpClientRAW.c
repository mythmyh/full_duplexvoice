/*
 ***************************************************************************************************************
 ***************************************************************************************************************
 ***************************************************************************************************************

 File:		  	   tcpClientRAW.c
 Modified By:     ControllersTech.com
 Updated:    	   29-Jul-2021

 ***************************************************************************************************************
 Copyright (C) 2017 ControllersTech.com

 This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
 of the GNU General Public License version 3 as published by the Free Software Foundation.
 This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
 or indirectly by this software, read more about this on the GNU General Public License.

 ***************************************************************************************************************
 */

/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable  application.
 *
 **/

/* This file was modified by ST */
#include<stdlib.h>
#include "tcpClientRAW.h"
#include "i2s.h"
#include <stdio.h>
#include <string.h>
#include "lwip/tcp.h"
#include <math.h>
#define BUFFSIZE 970

int recv_index = 0;
long a1 = 0, b1 = 0;
#define I2S_LEN 2
#define ARRLEN (1000/I2S_LEN/2)
#define STEP  (I2S_LEN*2)
volatile int next = 0;
//extern uint8_t abc[1280];
int resend_no;

/*  protocol states */
enum tcp_client_states {
	ES_NONE = 0, ES_CONNECTED, ES_RECEIVING, ES_CLOSING
};

uint32_t bytesread;

//uint32_t dcmi_data_buff[16000] = { 0 };
//11328

typedef struct {
	uint32_t ChunkID;		   	//chunk id;ÕâÀï¹Ì¶¨Îª"RIFF",¼´0X46464952
	uint32_t ChunkSize;		   	//¼¯ºÏ´óÐ¡;ÎÄ¼þ×Ü´óÐ¡-8
	uint32_t Format;	   			//¸ñÊ½;WAVE,¼´0X45564157
} ChunkRIFF;
//fmt¿é
typedef struct {
	uint32_t ChunkID;		   	//chunk id;ÕâÀï¹Ì¶¨Îª"fmt ",¼´0X20746D66
	uint32_t ChunkSize;		   	//×Ó¼¯ºÏ´óÐ¡(²»°üÀ¨IDºÍSize);ÕâÀïÎª:20.
	uint16_t AudioFormat;	  	//ÒôÆµ¸ñÊ½;0X01,±íÊ¾ÏßÐÔPCM;0X11±íÊ¾IMA ADPCM
	uint16_t NumOfChannels;		//Í¨µÀÊýÁ¿;1,±íÊ¾µ¥ÉùµÀ;2,±íÊ¾Ë«ÉùµÀ;
	uint32_t SampleRate;			//²ÉÑùÂÊ;0X1F40,±íÊ¾8Khz
	uint32_t ByteRate;			//×Ö½ÚËÙÂÊ;
	uint16_t BlockAlign;			//¿é¶ÔÆë(×Ö½Ú);
	uint16_t BitsPerSample;		//µ¥¸ö²ÉÑùÊý¾Ý´óÐ¡;4Î»ADPCM,ÉèÖÃÎª4
//	uint16_t ByteExtraData;		//¸½¼ÓµÄÊý¾Ý×Ö½Ú;2¸ö; ÏßÐÔPCM,Ã»ÓÐÕâ¸ö²ÎÊý
} ChunkFMT;
//fact¿é
typedef struct {
	uint32_t ChunkID;		   	//chunk id;ÕâÀï¹Ì¶¨Îª"fact",¼´0X74636166;
	uint32_t ChunkSize;		   	//×Ó¼¯ºÏ´óÐ¡(²»°üÀ¨IDºÍSize);ÕâÀïÎª:4.
	uint32_t NumOfSamples;	  	//²ÉÑùµÄÊýÁ¿;
} ChunkFACT;
//LIST¿é
typedef struct {
	uint32_t ChunkID;		   	//chunk id;ÕâÀï¹Ì¶¨Îª"LIST",¼´0X74636166;
	uint32_t ChunkSize;		   	//×Ó¼¯ºÏ´óÐ¡(²»°üÀ¨IDºÍSize);ÕâÀïÎª:4.
} ChunkLIST;

//data¿é
typedef struct {
	uint32_t ChunkID;		   	//chunk id;ÕâÀï¹Ì¶¨Îª"data",¼´0X5453494C
	uint32_t ChunkSize;		   	//×Ó¼¯ºÏ´óÐ¡(²»°üÀ¨IDºÍSize)
} ChunkDATA;
typedef struct {
	ChunkRIFF riff;	//riff¿é
	ChunkFMT fmt;  	//fmt¿é
//	ChunkFACT fact;	//fact¿é ÏßÐÔPCM,Ã»ÓÐÕâ¸ö½á¹¹Ìå
	ChunkDATA data;	//data¿é
} __WaveHeader;


int file_index = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* structure for maintaining connection infos to be passed as argument
 to LwIP callbacks*/
struct tcp_client_struct {
	u8_t state; /* current connection state */
	u8_t retries;
	struct tcp_pcb *pcb; /* pointer on the current tcp_pcb */
	struct pbuf *p; /* pointer on the received/to be transmitted pbuf */
};

/* This callback will be called, when the client is connected to the server */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

/* This callback will be called, when the client receive data from the server */
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p,
		err_t err);

/* This callback will be called, when the server Polls for the Client */
static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb);
void resend(int counter);
/* This callback will be called, when the server acknowledges the data sent by the client */
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);

/* A Function to send the data to the server */
static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_struct *es);
static void tcp_client_send2(struct tcp_pcb *tpcb, uint8_t *ptr);

/* Function to close the connection */
static void tcp_client_connection_close(struct tcp_pcb *tpcb,
		struct tcp_client_struct *es);

/* This is the part where we are going to handle the incoming data from the server */
static void tcp_client_handle(struct tcp_pcb *tpcb,
		struct tcp_client_struct *es);
void echo(void);
void send_poolsize(int index);
static void tcp_err2(void *arg, err_t err);

volatile int counter = 0;
volatile int len;
volatile int buff1_can_send = 0;
volatile int buff2_can_send = 0;

/* create a struct to store data */
static struct tcp_client_struct *esTx = 0;
static struct tcp_pcb *pcbTx = 0;
int numarr[2];
volatile int server_index = 0;
int current_index = 0;
int current_end = 10;
int server_direction = 1;
extern int direction;
volatile int record_index = 0;
int record_direction = 1;
uint8_t i2srecbuf1[BUFFSIZE * 4];
uint16_t i2splaybuf[2] = { 0X0000, 0X0000 };
uint8_t i2srecbuf2[BUFFSIZE * 4];
//uint8_t recvsram[1024000] __attribute__((section(".sram")));
uint8_t recvsram[34920];

extern uint8_t wavsram[11640];
//uint8_t wavsram[970 * 21];
extern int wav_index;
extern int start;
int start = 0;
uint32_t bytesread;
uint32_t write;
int all_index = 0;
int first_pack = 1;
int compensation = 0;
int play_index = 0;
char UsbDiskPath2[4] = { 0 };
void rec_i2s_dma_rx_callback(void) {

//	printf("cccc\n");
	b1 += 4;
	if (DMA1_Stream3->CR & (1 << 19)) {
		memcpy(wavsram + BUFFSIZE * record_index * 4, i2srecbuf1,
				sizeof(i2srecbuf1));
	} else {
		memcpy(wavsram + BUFFSIZE * record_index * 4, i2srecbuf2,
				sizeof(i2srecbuf2));
	}
	record_index += 1;
	if (record_index >= 3)
		record_index = 0;

}

static void tcp_err2(void *arg, err_t err) {

	printf("connect error,!closed by core ");

}

//void echo()
//{
//
//
//		//if(total_time>0)
//
//
//	if(echo_run==1){
//
//
//	//	printf("start send %d.bmp,time\r\n",circle_time);
//		echo_run=0;
//
//	esTx->p = pbuf_alloc(PBUF_RAW,3840, PBUF_POOL);
//	if(esTx->p!=NULL){
//	pbuf_take(esTx->p,abc,3840);
//	tcp_client_send(pcbTx, esTx);
//	pbuf_free(esTx->p);
//	}
//
//
//
//
//}
//}

//void send_poolsize(int counter) {
//   // retSD = f_read(&fil, rtext, sizeof(rtext), (UINT*)&bytesread);		//读取文件内容放到rtext�?
//	//f_lseek(&file,counter*4096);
//	int ret =f_read(&file, testsram,4096, (UINT*)&bytesread);
//	printf("result ret%d\n",ret);
//	//printf("counter=%d\r\n",counter*4096);
//	for (int i = 0; i < 3; i++) {
//		esTx->p = pbuf_alloc(PBUF_RAW, read_list[i], PBUF_POOL);
//		if (esTx->p != NULL) {
//			pbuf_take(esTx->p, testsram+1400*i, read_list[i]);
//			tcp_client_send(pcbTx, esTx);
//			pbuf_free(esTx->p);
//		}
//	}
//
//}

void send_poolsize(int index) {

	//uint8_t *payload_data = (uint8_t *)p->payload;
	//uint8_t second_element = payload_data[2]

	//printf("-----\n");

	if(recv_index-play_index>=6){
		compensation=0;
	}else{
		compensation=6-(recv_index-play_index);
		if(compensation>6)compensation=6;
		printf("compensatin %d\n",compensation);
	}
	esTx->p = pbuf_alloc(PBUF_RAW, BUFFSIZE, PBUF_POOL);
	if (esTx->p != NULL) {
		//	pbuf_take(esTx->p, wavsram, BUFFSIZE);
		pbuf_take(esTx->p, wavsram + BUFFSIZE * index, BUFFSIZE);

		uint8_t *payload_data = (uint8_t *)esTx->p->payload;
			payload_data[1]=compensation;

		tcp_client_send(pcbTx, esTx);
		pbuf_free(esTx->p);
	}

	//all_index += 1;

//	if (DMA1_Stream3->CR & (1 << 19)) {
//
//
//			tcp_client_send2(pcbTx, i2srecbuf1);
//
//
//
//	} else {
//		tcp_client_send2(pcbTx, i2srecbuf2);
//
//	}

}

void recoder_wav_init(__WaveHeader* wavhead) //³õÊ¼»¯WAVÍ·
{
	wavhead->riff.ChunkID = 0X46464952;	//"RIFF"
	wavhead->riff.ChunkSize = 0;			//»¹Î´È·¶¨,×îºóÐèÒª¼ÆËã
	wavhead->riff.Format = 0X45564157; 	//"WAVE"
	wavhead->fmt.ChunkID = 0X20746D66; 	//"fmt "
	wavhead->fmt.ChunkSize = 16; 			//´óÐ¡Îª16¸ö×Ö½Ú
	wavhead->fmt.AudioFormat = 0X01; 		//0X01,±íÊ¾PCM;0X01,±íÊ¾IMA ADPCM
	wavhead->fmt.NumOfChannels = 2;		//Ë«ÉùµÀ
	wavhead->fmt.SampleRate = 16000;		//16Khz²ÉÑùÂÊ ²ÉÑùËÙÂÊ
	wavhead->fmt.ByteRate = wavhead->fmt.SampleRate * 4;//×Ö½ÚËÙÂÊ=²ÉÑùÂÊ*Í¨µÀÊý*(ADCÎ»Êý/8)
	wavhead->fmt.BlockAlign = 4;			//¿é´óÐ¡=Í¨µÀÊý*(ADCÎ»Êý/8)
	wavhead->fmt.BitsPerSample = 16;		//16Î»PCM
	wavhead->data.ChunkID = 0X61746164;	//"data"
	wavhead->data.ChunkSize = 0;			//Êý¾Ý´óÐ¡,»¹ÐèÒª¼ÆËã
}

/* IMPLEMENTATION FOR TCP CLIENT

 1. Create TCP block.
 2. connect to the server
 3. start communicating
 */

void tcp_client_init(void) {
	/* 1. create new tcp pcb */
	struct tcp_pcb *tpcb;

	tpcb = tcp_new();

	/* 2. Connect to the server */
	ip_addr_t destIPADDR;
	IP_ADDR4(&destIPADDR, 192, 168, 1, 28);
	//while(ok!= ERR_OK)

	tcp_connect(tpcb, &destIPADDR, 12345, tcp_client_connected);
	HAL_Delay(5000);
	printf("tpc address = %p\r\n,", tpcb);

	I2S2_Init(I2S_STANDARD_PHILIPS, I2S_MODE_MASTER_TX, I2S_CPOL_LOW,
	I2S_DATAFORMAT_16B);	//飞利浦标�???????,主机发�??,时钟低电平有�???????,16位帧长度
	I2S2_SampleRate_Set(16000);	//设置采样�???????
	I2S2_TX_DMA_Init((uint8_t*) &i2splaybuf[0], (uint8_t*) &i2splaybuf[1], 1); //配置TX DMA
	DMA1_Stream4->CR &= ~(1 << 4);	//关闭传输完成中断(这里不用中断送数�???????)
	I2S2ext_RX_DMA_Init(i2srecbuf1, i2srecbuf2, BUFFSIZE * 2); 		//配置RX DMA
	i2s_rx_callback = rec_i2s_dma_rx_callback;
	I2S_Play_Start();
	I2S_Rec_Start();

//	__WaveHeader *wavhead = (__WaveHeader*) malloc(sizeof(__WaveHeader ));
//	recoder_wav_init(wavhead);
//	f_open(&file, file_name2, (FA_CREATE_ALWAYS | FA_WRITE));
//	printf("open wav %d\n", res2);
//	res2 = f_write(&file, wavhead, sizeof(__WaveHeader ), (void*) &write);
//	printf("write wav %d\n", res2);
//	res2=f_open(&wavfile, wavname, FA_READ);
//	printf("open %d\n",res2);

	tcp_err(tpcb, tcp_err2);

}

/** This callback is called, when the client is connected to the server
 * Here we will initialise few other callbacks
 * and in the end, call the client handle function
 */
static err_t tcp_client_connected(void *arg, struct tcp_pcb *newpcb, err_t err) {
	err_t ret_err;
	struct tcp_client_struct *es;

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	if (err == ERR_OK) {
		printf("TCP connection established successfully!\n");
	} else {
		printf("TCP connection failed with error code: %d\n", err);
	}

	/* allocate structure es to maintain tcp connection information */
	es = (struct tcp_client_struct*) mem_malloc(
			sizeof(struct tcp_client_struct));
	if (es != NULL) {
		es->state = ES_CONNECTED;
		es->pcb = newpcb;
		es->retries = 0;
		es->p = NULL;

		/* pass newly allocated es structure as argument to newpcb */
		tcp_arg(newpcb, es);

		/* initialize lwip tcp_recv callback function for newpcb  */
		tcp_recv(newpcb, tcp_client_recv);

		/* initialize lwip tcp_poll callback function for newpcb */
		tcp_poll(newpcb, tcp_client_poll, 0);

		/* initialize LwIP tcp_sent callback function */
		tcp_sent(newpcb, tcp_client_sent);
		/* handle the TCP data */
		tcp_client_handle(newpcb, es);

		ret_err = ERR_OK;
	} else {
		/*  close tcp connection */
		tcp_client_connection_close(newpcb, es);
		/* return memory error */
		ret_err = ERR_MEM;
	}
	return ret_err;
}

/** This callback is called, when the client receives some data from the server
 * if the data received is valid, we will handle the data in the client handle function
 */
static err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p,
		err_t err) {
	struct tcp_client_struct *es;
	err_t ret_err;

	LWIP_ASSERT("arg != NULL", arg != NULL);

	es = (struct tcp_client_struct*) arg;

	/* if we receive an empty tcp frame from server => close connection */
	if (p == NULL) {
		/* remote host closed connection */
		es->state = ES_CLOSING;
		if (es->p == NULL) {
			/* we're done sending, close connection */
			tcp_client_connection_close(tpcb, es);
		} else {
			/* we're not done yet */
//      /* acknowledge received packet */
//      tcp_sent(tpcb, tcp_client_sent);
			/* send remaining data*/
//      tcp_client_send(tpcb, es);
		}
		ret_err = ERR_OK;
	}
	/* else : a non empty frame was received from server but for some reason err != ERR_OK */
	else if (err != ERR_OK) {
		/* free received pbuf*/
		if (p != NULL) {
			es->p = NULL;
			pbuf_free(p);
		}
		ret_err = err;
	} else if (es->state == ES_CONNECTED) {
		/* store reference to incoming pbuf (chain) */
		//es->p = p;
		//	printf(" resend no %d\r\n",resend_no);
		// tcp_sent has already been initialized in the beginning.
//    /* initialize LwIP tcp_sent callback function */
		tcp_recved(tpcb, p->tot_len);
		if (recv_index == 1) {
			Audio_Player_Start();
			first_pack = 0;
		}

		if (server_index >= 36) {
			server_index = 0;
		}
		memcpy(recvsram + server_index * 970, p->payload, p->tot_len);
		recv_index += 1;
		server_index += 1;
	if (compensation == 0) {
			send_poolsize(current_index);
			current_index += 1;
			if (current_index == current_end) {
				if (record_index == 0) {
					current_index = 8;
					current_end = current_index + 4;
				} else {
					current_index = (record_index - 1) * 4;
					current_end = current_index + 4;
				}

			}

		}

		else {

			compensation--;
		}

//		if((server_index>1000)||(server_index<0)){
//
//			server_direction=-server_direction;
//		}

		/* Acknowledge the received data */

		/* handle the received data */
		pbuf_free(p);

		ret_err = ERR_OK;
	} else if (es->state == ES_CLOSING) {
		/* odd case, remote side closing twice, trash data */
		tcp_recved(tpcb, p->tot_len);
		es->p = NULL;
		pbuf_free(p);
		ret_err = ERR_OK;
	} else {
		/* unknown es->state, trash data  */
		tcp_recved(tpcb, p->tot_len);
		es->p = NULL;
		pbuf_free(p);
		ret_err = ERR_OK;
	}
	return ret_err;
}

static err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb) {
	err_t ret_err;
	struct tcp_client_struct *es;

	es = (struct tcp_client_struct*) arg;
	if (es != NULL) {
		if (es->p != NULL) {
			// tcp_sent has already been initialized in the beginning.
			tcp_sent(tpcb, tcp_client_sent);
			/* there is a remaining pbuf (chain) , try to send data */
//      tcp_client_send(tpcb, es);
		} else {
			/* no remaining pbuf (chain)  */
			if (es->state == ES_CLOSING) {
				/*  close tcp connection */
				tcp_client_connection_close(tpcb, es);
			}
		}
		ret_err = ERR_OK;
	} else {
		/* nothing to be done */
		tcp_abort(tpcb);
		ret_err = ERR_ABRT;
	}
	return ret_err;
}

/** This callback is called, when the server acknowledges the data sent by the client
 * If there is no more data left to sent, we will simply close the connection
 */
static err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
	struct tcp_client_struct *es;

	LWIP_UNUSED_ARG(len);
	//	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_9);

	es = (struct tcp_client_struct*) arg;
	es->retries = 0;

	if (es->p != NULL) {
		// tcp_sent has already been initialized in the beginning.
		/* still got pbufs to send */
//    tcp_client_send(tpcb, es);
	} else {
		/* if no more data to send and client closed connection*/
		if (es->state == ES_CLOSING)
			tcp_client_connection_close(tpcb, es);
	}
	return ERR_OK;
}

/** A function to send the data to the server
 */
static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_struct *es) {
	struct pbuf *ptr;

	/* get pointer on pbuf from es structure */
	ptr = es->p;

	tcp_write(tpcb, ptr->payload, ptr->len, 1);

	tcp_output(tpcb);
	//tcp_recved(tpcb, ptr->tot_len);

}

static void tcp_client_send2(struct tcp_pcb *tpcb, uint8_t *ptr) {

	/* get pointer on pbuf from es structure */

	tcp_write(tpcb, ptr, BUFFSIZE, 0);

	tcp_output(tpcb);
	//tcp_recved(tpcb, ptr->tot_len);

}

static void tcp_client_connection_close(struct tcp_pcb *tpcb,
		struct tcp_client_struct *es) {

	/* remove all callbacks */
	tcp_arg(tpcb, NULL);
	tcp_sent(tpcb, NULL);
	tcp_recv(tpcb, NULL);
	tcp_err(tpcb, NULL);
	tcp_poll(tpcb, NULL, 0);

	/* delete es structure */
	if (es != NULL) {
		mem_free(es);
	}

	/* close tcp connection */
	tcp_close(tpcb);
}

/* Handle the incoming TCP Data */

static void tcp_client_handle(struct tcp_pcb *tpcb,
		struct tcp_client_struct *es) {
	/* get the Remote IP */

	/* Extract the IP */

//	esTx->state = es->state;
//	esTx->pcb = es->pcb;
//	esTx->p = es->p;
	esTx = es;
	pcbTx = tpcb;

}
