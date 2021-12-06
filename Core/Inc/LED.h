/*
 * LED.h
 *
 *  Created on: Feb 17, 2020
 *      Author: carlr
 */

#ifndef SRC_LED_H_
#define SRC_LED_H_

void Record_LED();
void Playback_LED();
void Stop_LED();
void StopCount(int sec);

extern uint32_t Expired;
extern uint32_t expire;
extern uint32_t Counting;

#endif /* SRC_LED_H_ */
