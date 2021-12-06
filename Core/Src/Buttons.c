/*
 * LED.c
 *
 *  	Created on: 		February 17, 2020
 *      Author: 			Carl Rezelman
 *      Student Number:		19803036
 */

#include "main.h"
#include "Buttons.h"
#include "LED.h"
#include "stm32f4xx_it.h"
#include "sinewave.h"
#include "fatfs.h"
#include "fatfs_sd.h"
#include "stm32f4xx_hal.h"
#include "SD.h"

UART_HandleTypeDef huart2;
DAC_HandleTypeDef hdac;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim4;

volatile uint8_t Button_1 = 0;
volatile uint8_t Button_2 = 0;
volatile uint8_t Button_3 = 0;
volatile uint8_t Stop = 0;
volatile uint8_t Record = 0;
volatile uint8_t n = 0;

volatile uint8_t Recording = 0;
volatile uint8_t Playing = 0;
volatile uint8_t Stopped = 1;

FATFS fs;  // file system
FIL fil;  // file
FRESULT fresult;  // to store the result
FILINFO filinfo;
char buffer[1024]; // to store data
char filename[10];

BYTE work[_MAX_SS];

UINT br, bw;   // file read/write count

/* capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void Buttons()
{
	//Button 1
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) && Stopped)
	{
		Button_1 = 1;
		n = 1;
	}
	else
		Button_1 = 0;
	//Button 2
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) && Stopped)
	{
		Button_2 = 1;
		n = 2;
	}
	else
		Button_2 = 0;
	//Button 3
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) && Stopped)
	{
		Button_3 = 1;
		n = 3;
	}
	else
		Button_3 = 0;
	//Stop
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
		Stop = 1;
	else
		Stop = 0;
    //Record
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5))
		Record = 1;
	else
		Record = 0;
}

void Record_msg()
{

	//Button 1
	if(Record && Button_1 && !Recording && !Playing && Stopped && !Expired && n)
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
		{
			Button_1 = 0;
			Stopped = 0;
			Recording = 1;
			fresult = f_unlink("/file1.txt");

			sprintf(filename,"file%d.txt",n);
			fresult = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);

			uint8_t record_tx[10] = {127,128,'R','e','c','o','r','d','_',(n+48)};
			HAL_UART_Transmit(&huart2,record_tx,10,100);
			__HAL_TIM_ENABLE(&htim4);
			__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

			HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_Buffer, 1024);
		}
	}
	//Button 2
	if(Record && Button_2 && !Recording && !Playing && Stopped && !Expired && n)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == 0)
		{
			Button_2 = 0;
			Stopped = 0;
			Recording = 1;
			fresult = f_unlink("/file2.txt");

			sprintf(filename,"file%d.txt",n);
			fresult = f_open(&fil, filename, FA_OPEN_APPEND  | FA_WRITE);

			uint8_t record_tx[10] = {127,128,'R','e','c','o','r','d','_',(n+48)};
			HAL_UART_Transmit(&huart2,record_tx,10,100);
			__HAL_TIM_ENABLE(&htim4);
			__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

			HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_Buffer, 1024);
		}
	}
	//Button 3
	if(Record && Button_3 && !Recording && !Playing && Stopped && !Expired && n)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == 0)
		{
			Button_3 = 0;
			Stopped = 0;
			Recording = 1;
			fresult = f_unlink("/file3.txt");

			sprintf(filename,"file%d.txt",n);
			fresult = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);

			uint8_t record_tx[10] = {127,128,'R','e','c','o','r','d','_',(n+48)};
			HAL_UART_Transmit(&huart2,record_tx,10,100);
			__HAL_TIM_ENABLE(&htim4);
			__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

			HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_Buffer, 1024);
		}
	}
}

void Playback_msg()
{
	//Button 1
	if(Button_1 && !Record && !Recording && !Playing && Stopped)
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
		{
			Button_1 = 0;
			Stopped = 0;
			Playing = 1;

			uint8_t play_tx[10] = {127,128,'P','l','a','y','_','_','_',(n+48)};
			HAL_UART_Transmit(&huart2,play_tx,10,100);

			sprintf(filename,"file%d.txt",n);
			fresult = f_open(&fil, filename,FA_READ);
			fresult = f_read(&fil, DAC_Buffer, 1024, &br);
			HAL_UART_Transmit(&huart2,(uint8_t*)DAC_Buffer,1024,100);
			HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)&DAC_Buffer,1024,DAC_ALIGN_8B_R);
			Read_SD(1);

			//wave_fillbuffer(DAC_Buffer_1, 1, 1024);
		    //HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)&Play_1,44100,DAC_ALIGN_8B_R);
		}
	}
	//Button 2
	if(Button_2 && !Record && !Recording && !Playing && Stopped)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == 0)
		{
			Button_2 = 0;
			Stopped = 0;
			Playing = 1;

			uint8_t play_tx[10] = {127,128,'P','l','a','y','_','_','_',(n+48)};
			HAL_UART_Transmit(&huart2,play_tx,10,100);

			//Read_SD(2);

			//wave_fillbuffer(DAC_Buffer_2, 2, 1024);
		   // HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)&Play_2,44100,DAC_ALIGN_8B_R);
		}
	}
	//Button 3
	if(Button_3 && !Record && !Recording && !Playing && Stopped)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == 0)
		{
			Button_3 = 0;
			Stopped = 0;
			Playing = 1;

			uint8_t play_tx[10] = {127,128,'P','l','a','y','_','_','_',(n+48)};
			HAL_UART_Transmit(&huart2,play_tx,10,100);

			//Read_SD(3);

			//wave_fillbuffer(DAC_Buffer_3, 3, 1024);
		    //HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)&Play_3,44100,DAC_ALIGN_8B_R);
		}
	}
}

void Stop_msg()
{
	//Send Stop Message
	if((Stop && (Playing || Recording)) || Expired)// || eof || no_file)
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == 0)
		{

			Stop = 0;
			Playing = 0;
			Recording = 0;
			Counting = 0;
			Expired = 0;
			Stopped = 1;
			eof = 0;
			no_file = 0;

			uint8_t stop_tx[10] = {127,128,'S','t','o','p','_','_','_','_'};
			HAL_UART_Transmit(&huart2,stop_tx,10,100);

			//fresult = f_write(&fil, Transmit, 1024*i, &bw);
			HAL_DAC_Stop_DMA(&hdac,DAC_CHANNEL_1);
			HAL_ADC_Stop_DMA(&hadc1);
			f_close(&fil);
			n = 0;
			i = 0;

		//	open = 1;
		//	Drive_info();
		}
	}
}

