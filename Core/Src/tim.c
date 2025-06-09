/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
extern volatile uint16_t CntRx1;
extern uint16_t USART1_REC_STA;
extern uint16_t Uart1_ReceiveData_flag;
extern volatile uint16_t CntRx2;
extern uint16_t USART2_REC_STA;
extern uint16_t Uart2_ReceiveData_flag;
extern volatile bool sendOilPumpDataFlag;
extern uint32_t OilPumpDataCount;
extern UART_HandleTypeDef huart1;

extern void TaskRemarks(void);

//写入占空比
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3)
{
  /* Check the parameters */
  assert_param(IS_TIM_LIST8_PERIPH(TIMx));
  /* Set the Capture Compare1 Register value */
  TIM1->CCR1 = Compare3;
	TIM1->CCR2 = Compare3;
	TIM1->CCR3 = Compare3;
	TIM1->CCR4 = Compare3;

}

//定时器中断处理函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//定时器2定1ms中断
	if(htim==(&htim2))
  {
		if(sendOilPumpDataFlag){
			OilPumpDataCount += 1;
		}
		TaskRemarks();
		if(CntRx1)				    
		{
			if(--CntRx1==0)	    	    //计数器减到0 ，7ms定时到
			{
				USART1_REC_STA|=1<<15;   //标志为接收完成
				Uart1_ReceiveData_flag = 1;
			}
		}
		if(CntRx2)				    
		{
			if(--CntRx2==0)	    	    //计数器减到0 ，7ms定时到
			{
				USART2_REC_STA|=1<<15;   //标志为接收完成
				Uart2_ReceiveData_flag = 1;
			}
		}
		__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
  }
}

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */
	//开启定时器中断----->1ms
//		HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


/* USER CODE END 1 */
