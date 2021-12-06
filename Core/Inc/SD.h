/*
 * SD.h
 *
 *  Created on: 31 Mar 2020
 *      Author: carlr
 */

#ifndef INC_SD_H_
#define INC_SD_H_

void Write_SD(uint8_t write[1024], int length, int n);
void Read_SD(int n);
void Clear_files();
void Drive_info();
void send_uart (char *string);
int bufsize (char *buf);
void bufclear (void);

extern int open;
extern uint8_t eof;
extern uint8_t no_file;

#endif /* INC_SD_H_ */
