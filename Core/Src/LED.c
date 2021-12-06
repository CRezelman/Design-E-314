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

uint32_t expire = 0;
uint32_t Expired = 0;
uint32_t Counting = 0;

TIM_HandleTypeDef htim4;

void Record_LED()
{
	if(Recording)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);			//Record LED ON

		if((n == 1) && (Flag % 2 == 0))
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);		//LED 1 Blinking
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);		//LED 1 OFF

		if((n == 2) && (Flag % 2 == 0))
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);		//LED 2 Blinking
		else
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);		//LED 2 OFF

		if((n == 3) && (Flag % 2 == 0))
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,1);		//LED 3 Blinking
		else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);		//LED 3 OFF
	}
	else
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);			//Record LED OFF

}

void Playback_LED()
{
	if(Playing)
	{
		if((n == 1) && (Flag % 2 == 0))
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);		//LED 1 Blinking
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);		//LED 1 OFF

		if((n == 2) && (Flag % 2 == 0))
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1);		//LED 2 Blinking
		else
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);		//LED 2 OFF

		if((n == 3) && (Flag % 2 == 0))
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,1);		//LED 3 Blinking
		else
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);		//LED 3 OFF
	}
}

void Stop_LED()
{
	if (Stopped || Expired)
	{
		Expired = 0;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);		//LED 1 OFF
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);		//LED 2 OFF
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,0);		//LED 3 OFF
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);		//Record LED OFF
	}
}


void StopCount(int sec)
{
//	if (Recording && (n == 1 || n == 2 || n == 3) && !Counting) 	//Check counting hasn't started
//	{
//		expire = HAL_GetTick() + 1000*sec/7;					//Set expire time
//		Counting = 1;								//Start Counting
//	}
//
//	if ((HAL_GetTick() >= expire && HAL_GetTick() != 0) && expire != 0 )			//Compare expire time to actual time
//	{
//
//		Expired = 1;								//Set Expired Flag
//		expire = 0;
//		Counting = 0;								//Stop Counting
//	}

	if(Timer3 >= sec && Timer3 != 0)
	{
		Expired = 1;
		__HAL_TIM_DISABLE(&htim4);
		Timer3 = 0;
	}
}


