/*
 * Buttons.h
 *
 *  Created on: Feb 17, 2020
 *      Author: carlr
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

void Buttons();
void Record_msg();
void Playback_msg();
void Stop_msg();

extern volatile uint8_t Button_1;
extern volatile uint8_t Button_2;
extern volatile uint8_t Button_3;
extern volatile uint8_t Stop;
extern volatile uint8_t Record;
extern volatile uint8_t n;

extern volatile uint8_t Recording;
extern volatile uint8_t Playing;
extern volatile uint8_t Stopped;

extern char filename[10];

#endif /* INC_BUTTONS_H_ */
