/*
 * RED.c
 *
 *  Created on: Feb 28, 2025
 *      Author: zhang
 */
#include <RED.h>

/**
 * @brief 检测是否有人
 * @note 传感器有人时输出低电平（GPIOB0为RESET），无人时高电平（SET）
 */
bool hal_detect_Closeup_human(void)//近距离0-10mm
{
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==SET)
	{
		return false;//无人
	}
	else
	{
		return true;//有人
	}
}

bool hal_detect_Ldistance_human(void)//远距离10-500mm
{
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)==SET)
	{
		return true;//无人
	}
	else
	{
		return false;//有人
	}
}
