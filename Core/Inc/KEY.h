/*
 * KEY.h
 *
 *  Created on: Feb 28, 2025
 *      Author: zhang
 */

#ifndef INC_KEY_H_
#define INC_KEY_H_

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <PWM.h>
#include <RED.h>
#include <Bluetooth.h>
#include <adc.h>
#include <KEY.h>
#include <oled.h>
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

void Timer_Control(void);
void Manual_Control(void);
void Auto_Control(void);
int Apply_Sliding_Filter(int new_value);
void key_control(void);

#endif /* INC_KEY_H_ */
