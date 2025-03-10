/*
 * PWM.c
 *
 *  Created on: Feb 28, 2025
 *      Author: zhang
 */
#include "tim.h"
#include <PWM.h>

#define PWM_SCALE_FACTOR   10    // 精度放大倍数（0.1%步进）
#define PWM_MAX_VALUE      999   // 99.9%对应整数值（999 = 99.9 * 10）

extern uint32_t led;

/*确定范围*/
void hal_ledpwm(uint32_t a)
  {
	  if (a<=10)
	  {
		  led=0;
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	  }
	  else if (a>=90)
	  {
		  led=99.9;
		  // 直接设置比较值，无需重新初始化
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led);//调pwm波暗
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 确保PWM已启动
	  }
	  else if(a>10&&a<90)
	  {
		  led=a;
		  // 直接设置比较值，无需重新初始化
		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led);//调pwm波暗
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // 确保PWM已启动
	  }
  }
