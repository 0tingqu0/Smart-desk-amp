/*
 * PWM.c
 *
 *  Created on: Feb 28, 2025
 *      Author: zhang
 */
#include "tim.h"
#include <PWM.h>

extern uint32_t led;

/*确定范围*/
void hal_ledpwm(uint32_t a)
  {
	  if (a<=0)
	  {
		  led=0;
		  MX_TIM1_Init();
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	  }
	  else if (a>=100)
	  {
		  led=99.9;
		  MX_TIM1_Init();
		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//输出tim1的pwm波
	  }
	  else if(a>0&&a<100)
	  {
		  led=a;
		  MX_TIM1_Init();
		  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//输出tim1的pwm波
	  }
  }
