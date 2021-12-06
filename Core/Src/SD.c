/*
 * SD.c
 *
 *  Created on: 31 Mar 2020
 *      Author: carlr
 */

#include "main.h"
#include "fatfs.h"
#include "fatfs_sd.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "Buttons.h"

int open = 1;
uint8_t eof = 0;
uint8_t no_file = 0;

UART_HandleTypeDef huart2;
DAC_HandleTypeDef hdac;

FATFS fs;  // file system
FIL fil;  // file
FRESULT fresult;  // to store the result
FILINFO filinfo;
char buffer[1024]; // to store data

BYTE work[_MAX_SS];

UINT br, bw;   // file read/write count

/* capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void send_uart (char *string)
{
	uint8_t len = strlen (string);
	HAL_UART_Transmit(&huart2, (uint8_t *) string, len, 2000);  // transmit in blocking mode
}

/* to find the size of data in the buffer */
int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void bufclear (void)  // clear buffer
{
	for (int i=0; i<1024; i++)
	{
		buffer[i] = '\0';
	}
}

void Clear_files()
{
	fresult = f_unlink("/file1.txt");
	fresult = f_unlink("/file2.txt");
	fresult = f_unlink("/file3.txt");
}

void Drive_info()
{
	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf (buffer, "\nSD CARD Total Size: \t%lu KB\n",total);
	send_uart(buffer);
	bufclear();
	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
	sprintf (buffer, "SD CARD Free Space: \t%lu KB\n",free_space);
	send_uart(buffer);

}

void Write_SD(uint8_t write[1024], int length, int n)
{
	char filename[9];
	sprintf(filename,"file%d.txt",n);

	if(open != FR_OK)
		open = f_open(&fil, filename, FA_OPEN_APPEND | FA_READ | FA_WRITE);

	fresult = f_write(&fil, write, length, &bw);

}

void Read_SD(int n)
{
	uint8_t read[1024] = {0};
	char filename[9];
	sprintf(filename,"file%d.txt",n);

	if(open != FR_OK)
		open = f_open(&fil, filename,FA_READ);

	if(f_stat(filename, &filinfo) == FR_OK)
	{
		fresult = f_read(&fil, read, 1024, &br);

		eof = f_eof(&fil);

		HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_1,(uint32_t*)&read,1024,DAC_ALIGN_8B_R);
		memcpy(Transmit,read,1024);
	}
//	else
//		no_file = 1;





}


