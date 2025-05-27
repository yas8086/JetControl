/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// 任务结构
typedef struct _TASK_COMPONENTS
{
  uint8_t Run;                 				     // 程序运行标记：0-不运行，1运行
  uint16_t Timer;             					   // 计时器
  uint16_t ItvTime;              					 // 任务运行间隔时间
  void (*TaskHook)(void);   						   // 要运行的任务函数
}TASK_COMPONENTS;       							     // 任务定义

//// 任务清单
//typedef enum _TASK_LIST
//{
//  TASK_USART1_RESPOND,            	//串口1接收数据并处理
//  TASK_ENGINE_CONTROL,             	//串口2向发动机定时发送数据
//  // 这里添加你的任务。。。。
//  TASKS_MAX         // 总的可供分配的定时任务数目
//} TASK_LIST;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYSTEM_LED_Pin GPIO_PIN_0
#define SYSTEM_LED_GPIO_Port GPIOC
#define CARD_LED_Pin GPIO_PIN_1
#define CARD_LED_GPIO_Port GPIOC
#define OIL_PUMP_Pin GPIO_PIN_8
#define OIL_PUMP_GPIO_Port GPIOC
#define WATER_PUMP_Pin GPIO_PIN_9
#define WATER_PUMP_GPIO_Port GPIOC
#define POWDER_PUMP_Pin GPIO_PIN_8
#define POWDER_PUMP_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define MIN_PWM_PUMP1           			 0     		 //粉阀最小开启
#define MAX_PWM_PUMP1            			 100   		 //粉阀最大开启
#define USART1_MAX_RECV_LEN            20
#define USART2_MAX_RECV_LEN            10
#define TASKS_MAX                      2

#define FUNCTION_TEXT         0//功能测试代码（1为开，0为关）
#define OIL_PUMP_ON   		 		HAL_GPIO_WritePin(OIL_PUMP_GPIO_Port,OIL_PUMP_Pin,GPIO_PIN_RESET)       //油泵开  低电平开
#define OIL_PUMP_OFF 		      HAL_GPIO_WritePin(OIL_PUMP_GPIO_Port,OIL_PUMP_Pin,GPIO_PIN_SET)				//油泵关
//#define WATER_PUMP_ON        	HAL_GPIO_WritePin(OIL_PUMP_GPIO_Port,OIL_PUMP_Pin,GPIO_PIN_RESET)     //水泵开
//#define WATER_PUMP_OFF 		    HAL_GPIO_WritePin(OIL_PUMP_GPIO_Port,OIL_PUMP_Pin,GPIO_PIN_SET)		//水泵关
#define WATER_PUMP_ON        	HAL_GPIO_WritePin(WATER_PUMP_GPIO_Port,WATER_PUMP_Pin,GPIO_PIN_RESET)     //水泵开
#define WATER_PUMP_OFF 		    HAL_GPIO_WritePin(WATER_PUMP_GPIO_Port,WATER_PUMP_Pin,GPIO_PIN_SET)		//水泵关
#define POWDER_PUMP_ON        	HAL_GPIO_WritePin(POWDER_PUMP_GPIO_Port,POWDER_PUMP_Pin,GPIO_PIN_RESET)     //粉阀开
#define POWDER_PUMP_OFF 		    HAL_GPIO_WritePin(POWDER_PUMP_GPIO_Port,POWDER_PUMP_Pin,GPIO_PIN_SET)				//粉阀关
//#define PWM_PUMP1_ON   		 		HAL_GPIO_WritePin(PWM_PUMP1_GPIO_Port,PWM_PUMP1_Pin,GPIO_PIN_RESET)     //粉阀开
//#define PWM_PUMP1_OFF         HAL_GPIO_WritePin(PWM_PUMP1_GPIO_Port,PWM_PUMP1_Pin,GPIO_PIN_SET)       //粉阀关
#define SYSTEM_LED_ON        HAL_GPIO_WritePin(SYSTEM_LED_GPIO_Port,SYSTEM_LED_Pin,GPIO_PIN_RESET)     //系统LED开
#define SYSTEM_LED_OFF        HAL_GPIO_WritePin(SYSTEM_LED_GPIO_Port,SYSTEM_LED_Pin,GPIO_PIN_RESET)     //系统LED关

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
