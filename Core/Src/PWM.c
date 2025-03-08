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
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	  }
	  else if (a>=100)
	  {
		  led=99.9;
		  // 直接设置比较值，无需重新初始化
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led);//调pwm波暗
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 确保PWM已启动
	  }
	  else if(a>0&&a<100)
	  {
		  led=a;
		  // 直接设置比较值，无需重新初始化
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led);//调pwm波暗
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 确保PWM已启动
	  }
  }
