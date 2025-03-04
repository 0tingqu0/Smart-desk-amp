/*
 * photoresistor.c
 *
 *  Created on: Feb 28, 2025
 *      Author: zhang
 */
#include <photoresistor.h>
#include "stdio.h"
#include "usart.h"
#include "adc.h"

uint16_t ADC_Sample = 0,ADC_Volt = 0;//ADC_Value为采样值，ADC_Volt为电压值
uint8_t str[64];//给定一个数组空间，存放sprintf的内容


void UR3_Send_Info()
{
	sprintf((char*)str,"\r\nSampling value:%d,Voltage value:%d.%d%d",ADC_Sample,ADC_Volt/100,(ADC_Volt/10)%10,ADC_Volt%10);//使用sprintf把将要发送的内容存放到数组
	HAL_UART_Transmit(&huart3,str,sizeof(str),10000);//将数组中的内容发送到串口
}

void Get_ADC_Sample()
{
	HAL_ADC_Start(&hadc1);//打开ADC转换
	if(HAL_ADC_PollForConversion(&hadc1,10) == HAL_OK)
	{
		ADC_Sample = HAL_ADC_GetValue(&hadc1);//将得到的ADC采样值放入变量ADC_Sample中
		ADC_Volt = ADC_Sample * 330/4096;//数据转换，电压为3.3V，数据时12位，保留两位小数
	}
	UR3_Send_Info();//将上面数据进行存放，发送到上位机
	HAL_ADC_Stop(&hadc1);//停止ADC转换
}
