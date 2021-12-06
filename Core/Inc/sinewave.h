/*
 * sinewave.h
 *
 *  Created on: Mar 6, 2020
 *      Author: carlr
 */

#ifndef INC_SINEWAVE_H_
#define INC_SINEWAVE_H_

#include <stdint.h>

void wave_init(void);
void wave_fillbuffer(uint16_t* buffer, uint8_t type, uint16_t len);

#endif /* INC_SINEWAVE_H_ */
