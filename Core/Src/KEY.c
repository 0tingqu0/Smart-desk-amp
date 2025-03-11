/*
 * KEY.c
 *
 *  Created on: Feb 28, 2025
 *      Author: zhang
 */
#include <KEY.h>

#define SLIDING_WINDOW_SIZE   16   // 滑动窗口大小
int value = 0;
double voltage = 0.0;
char message[64] = "";
uint32_t i = 0;

long int mean_value = 0;
double mean_voltage = 0.0;

// 滑动窗口滤波相关变量
int filter_buffer[SLIDING_WINDOW_SIZE] = { 0 };
long int filter_sum = 0;
uint8_t filter_index = 0;
uint8_t led_state = 0;

//定时变量
uint32_t Tim_time = 3600; //初始1小时

// 全局变量定义
volatile uint8_t mode = 0; // 0:手动, 1:自动, 2:定时
extern uint32_t led;
extern uint8_t led_state;
extern uint8_t red_state;

uint8_t key_con = 0;
/*
 * 手动
 */
void Manual_Control(void)
{
    if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET) //key1 开关
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET && led_state == 0)
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET);
            hal_ledpwm(50);
            led_state = 1;
            key_con = 1;
        }
        else if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET && led_state == 1)
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET);
            hal_ledpwm(0);
            led_state = 0;
            key_con = 1;
        }
    }

    if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == GPIO_PIN_RESET) //key2 模式切换
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == GPIO_PIN_RESET)
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == GPIO_PIN_RESET);

            mode = (mode == 2) ? 0 : mode + 1;
            switch (mode)
            {
                case 0: // 手动模式
                    key_control();
                    break;
                case 1: // 自动模式
                    Auto_Control();

                    break;
                case 2: // 定时模式
                    Timer_Control();

                    break;
            }
        }

    }
}
/*
 * 滑动窗口滤波
 */
int Apply_Sliding_Filter(int new_value)
{
    filter_sum -= filter_buffer[filter_index];
    filter_buffer[filter_index] = new_value;
    filter_sum += new_value;
    filter_index = (filter_index + 1) % SLIDING_WINDOW_SIZE;
    return filter_sum / SLIDING_WINDOW_SIZE;
}
/*
 * 自动
 */
void Auto_Control(void)
{
    // 启动ADC转换并等待完成
    HAL_ADC_Start(&hadc1);  // 单次模式需每次启动
    //Enter_LowPower_Mode();//低功耗会导致无法烧录

    if (HAL_ADC_PollForConversion(&hadc1 , 100) == HAL_OK)
    {
        value = Apply_Sliding_Filter(HAL_ADC_GetValue(&hadc1));
        mean_value += value;
        voltage = (value / 4095.0) * 3.3;
        mean_voltage += voltage;
        i++;

        if (i == 32)    //可改1023
        {

            mean_value /= i;
            mean_voltage /= i;

            led = (mean_value * 100 / 4095);

//            snprintf(message , sizeof(message) , "ADC:%ld V:%.2f LED:%d MODE:%d" , mean_value , (float) mean_voltage , led , mode); // 直接发送数据
//
//            HAL_UART_Transmit(&huart1 , (uint8_t*) message , strnlen(message , sizeof(message)) , 100);

            if ((hal_detect_Closeup_human() && led_state == 0) || led_state == 1)       //判断人
            {
                HAL_ADC_Start_IT(&hadc1);

                hal_ledpwm(led);
                led_state = 1;
            }

            if (led_state == 1 && red_state == 0)
            {

                hal_ledpwm(0);      //调pwm波
                led_state = 0;
//                               HAL_NVIC_SystemReset();  // HAL库封装的系统复位函数[9](@ref)
            }

            i = 0;
            mean_value = 0;
            mean_voltage = 0.0;
        }
    }
    else
    {
        Error_Handler();
    }
}

/*
 * 定时
 */
void Timer_Control(void)
{
    if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == RESET) //key3 亮
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == RESET) //key3 亮
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == RESET);
            Tim_time += 600;
        }
    }

    else if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == RESET) //key4 暗
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == RESET) //key4 暗
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == RESET);
            Tim_time -= 600;
        }

    }
    else if (HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_11) == RESET) //key5 确定
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_11) == RESET)
        {
            while (HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_11) == RESET);
            AdjustTimerDuration(Tim_time);
            HAL_GPIO_WritePin(GPIOC , GPIO_PIN_13 , RESET);
        }
    }
}

/*
 * key1 开关
 * key2 模式切换
 * key3 亮
 * key4 暗
 * key5 确定
 */
void key_control(void)
{

    if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET) //key1 开关
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET && led_state == 0)
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET);
            hal_ledpwm(50);
            led_state = 1;
            key_con = 1;
        }
        else if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET && led_state == 1)
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_12) == GPIO_PIN_RESET);
            hal_ledpwm(0);
            led_state = 0;
            key_con = 1;
        }
    }
    else if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == GPIO_PIN_RESET) //key2 模式切换
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == GPIO_PIN_RESET)
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_13) == GPIO_PIN_RESET);

            mode = (mode == 2) ? 0 : mode + 1;
            switch (mode)
            {
                case 0: // 手动模式

                    break;
                case 1: // 自动模式
                    Auto_Control();

                    break;
                case 2: // 定时模式
                    Timer_Control();

                    break;
            }
        }

    }
    else if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == GPIO_PIN_RESET) //key3 亮
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == GPIO_PIN_RESET) //key3 亮
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) == GPIO_PIN_RESET);
            led += 10;
            hal_ledpwm(led);
        }
    }

    else if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == GPIO_PIN_RESET) //key4 暗
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == GPIO_PIN_RESET) //key4 暗
        {
            while (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) == GPIO_PIN_RESET);
            if (led == 0)
            {
                led_state = 0;
            }
            else
            {
                led = led - 10;
                hal_ledpwm(led);
            }
        }

    }
    else if (HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_11) == GPIO_PIN_RESET) //key5 确定
    {
        HAL_Delay(20);
        if (HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_11) == GPIO_PIN_RESET)
        {
            while (HAL_GPIO_ReadPin(GPIOA , GPIO_PIN_11) == GPIO_PIN_RESET);
            AdjustTimerDuration(Tim_time);
            HAL_GPIO_WritePin(GPIOC , GPIO_PIN_13 , GPIO_PIN_RESET);
        }
    }

}
