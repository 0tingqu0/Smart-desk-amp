/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <PWM.h>
#include <RED.h>
#include <Bluetooth.h>
#include <adc.h>
#include <KEY.h>
#include <oled.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 硬件配置宏定义
#define ENABLE_LOW_POWER      0  // 关闭低功耗

extern uint32_t led;
extern uint8_t red_state;
extern uint8_t led_state;
extern uint8_t key_con;
extern volatile uint8_t mode;
extern volatile uint8_t key5_state;
extern int value;
extern double voltage;
extern char message[64];
extern uint32_t i;
extern long int mean_value;
extern double mean_voltage;


volatile uint32_t Tim_time = 3610; //初始1小时
volatile uint32_t timerCounter = 0;     // 软件计数器（单位：秒）
volatile uint32_t targetCount = 10;   // 默认目标时间：3600秒=1小时
volatile uint32_t time_change=600;

volatile uint16_t rx_index = 0;  // 接收索引
volatile uint8_t frame_ready = 0;  // 帧完成标志

//接收区
//#define MAXSIZE 50
//#define RX3_BUF_SIZE 3
//#define RX1_BUF_SIZE 6

uint8_t uart3_rx_buf[8];
uint8_t uart1_rx_buf[8];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * RX回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        HAL_UART_Transmit_DMA(&huart1 , uart1_rx_buf , sizeof(uart1_rx_buf));
        if (uart1_rx_buf[0] == 0x40)
        {
            if (uart1_rx_buf[1] == 0x6F && uart1_rx_buf[2] == 0x70 && led_state == 0)
            {
                led = 50;
                hal_ledpwm(led); //调pwm波开灯
                led_state = 1;
                key_con = 1;
//                   memset(uart1_rx_buf,0,RX1_BUF_SIZE);
            }
            else if (uart1_rx_buf[1] == 0x63 && uart1_rx_buf[2] == 0x6c && led_state == 1)
            {
                led = 0;
                hal_ledpwm(led); //调pwm波关灯
                led_state = 0;
                key_con = 1;
//                   memset(uart1_rx_buf,0,RX1_BUF_SIZE);
            }
            else if (uart1_rx_buf[1] == 0x75 && uart1_rx_buf[2] == 0x70 && led_state == 1)
            {
                led +=5;
                hal_ledpwm(led);
//                   memset(uart1_rx_buf,0,RX1_BUF_SIZE);
            }
            else if (uart1_rx_buf[1] == 0x64 && uart1_rx_buf[2] == 0x6F && led_state == 1)
            {
                if (led == 0)
                {
                    led_state=0;
                }
                else
                {
                    led -=  5;
                }
                hal_ledpwm(led);
            }
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1 , uart1_rx_buf , sizeof(uart1_rx_buf));
    }

    if (huart->Instance == USART3)
    {
        // 检查起始字节是否为0xAA
        if (uart3_rx_buf[0] == 0xAA)
        {
            // 通过UART1非阻塞发送数据
            HAL_UART_Transmit_DMA(&huart1 , uart3_rx_buf , sizeof(uart3_rx_buf));
            if (uart3_rx_buf[1] == 0x01 && uart3_rx_buf[2] == 0x01 && led_state == 0)
            {
                led = 50;
                hal_ledpwm(led); //调pwm波开灯
                led_state = 1;
                key_con = 1;
//                memset(uart3_rx_buf, 0, 3);
            }
            else if (uart3_rx_buf[1] == 0x10 && uart3_rx_buf[2] == 0x10 && led_state == 1)
            {
                led = 0;
                hal_ledpwm(led); //调pwm波关灯
                led_state = 0;
                key_con = 1;
//                memset(uart3_rx_buf, 0, RX3_BUF_SIZE);

            }
            else if (uart3_rx_buf[1] == 0x10 && uart3_rx_buf[2] == 0x01 && led_state == 1)
            {
                led = led + 10;
                hal_ledpwm(led);
//                memset(uart3_rx_buf, 0, RX3_BUF_SIZE);
            }
            else if (uart3_rx_buf[1] == 0x01 && uart3_rx_buf[2] == 0x10 && led_state == 1)
            {
                if (led == 0)
                {
                    led_state=0;
                }
                else
                {
                    led = led - 10;
                }
                hal_ledpwm(led);
//                memset(uart3_rx_buf, 0, RX3_BUF_SIZE);
            }
        }

        // 重新启动接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3 , uart3_rx_buf , sizeof(uart3_rx_buf));
    }
}

/*
 * TX回调函数
 */
void HAL_UARTEx_TxEventCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        memset(uart1_rx_buf , 0 , sizeof(uart1_rx_buf));  // 发送完成后清空
//        HAL_UART_Transmit_IT(&huart1 , uart1_rx_buf , RX1_BUF_SIZE);
    }
    if (huart->Instance == USART3)
    {
        memset(uart3_rx_buf , 0 , sizeof(uart3_rx_buf));  // 发送完成后清空
//        HAL_UART_Transmit_IT(&huart3 , uart3_rx_buf , RX3_BUF_SIZE);
    }

}

/*
 *  在中断中加入误差补偿（单位：微秒）
 */
//static int32_t error_compensation = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if (htim->Instance == TIM2)
        {
            timerCounter++;                    // 每秒递增

            if (timerCounter >= targetCount)
            { // 达到目标时间
                timerCounter = 0;                // 重置计数器

                /* 用户自定义操作（示例：翻转LED） */
//                HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13); // 假设LED接在PC13
//                snprintf(message , sizeof(message) , "ADC:%ld V:%.2f i:%lu LED:%lu MODE:%u" , mean_value ,
//                        (float) mean_voltage , i , led , mode); // 直接发送数据
//                HAL_UART_Transmit(&huart1 , (uint8_t*) message , strnlen(message , sizeof(message)) , HAL_MAX_DELAY);
                if (key5_state == 1&&led_state==1)
                {
                    hal_ledpwm(0);
                    led_state = 0;
                    key5_state = 0;
                }

            }
        }
    }
}

/*
 * 动态调节定时时长
 */
void AdjustTimerDuration(uint32_t seconds)
{
    targetCount = seconds;    // 直接修改目标时间
    timerCounter = 0;          // 可选：重置当前计数
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    // 检查HSI时钟状态
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
    {
        Error_Handler();
    }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_ADCEx_Calibration_Start(&hadc1);



    HAL_UARTEx_ReceiveToIdle_DMA(&huart1 , uart1_rx_buf , sizeof(uart1_rx_buf));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3 , uart3_rx_buf , sizeof(uart3_rx_buf));

    HAL_TIM_Base_Start_IT(&htim2);

    HAL_ADC_Start_IT(&hadc1);

//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);  // 启用错误中断（包括ORE）
//    __HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);  // 启用错误中断（包括ORE）
//    HAL_NVIC_SystemReset();  // HAL库封装的系统复位函数[9](@ref)
//    __HAL_UART_ENABLE_IT(&huart1 , UART_IT_IDLE);  // 开启空闲中断
//    __HAL_UART_ENABLE_IT(&huart3 , UART_IT_IDLE);  // 开启空闲中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

        Manual_Control();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {

    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
