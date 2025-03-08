/*
 * KEY.c
 *
 *  Created on: Feb 28, 2025
 *      Author: zhang
 */
#include <KEY.h>

//int key1=0,key2=0,key3=0,key4=0,key5=0;
extern uint32_t led;

/*
 * key1 开关
 * key2 模式切换
 * key3 亮
 * key4 暗
 * key5 确定
 */
void key_control(){
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==RESET)//key1 开关
	{
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==RESET);
		hal_ledpwm(0);

	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==RESET)//key2 模式切换
	{
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==RESET);

	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==RESET)//key3 亮
	{
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==RESET);
		//led+=10;
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==RESET)//key4 暗
	{
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==RESET);
		//led-=10;
	}
	else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)==RESET)//key5 确定
	{
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)==RESET);

	}

}
