/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//应答：respond  接收：receive
volatile uint8_t Uart1_ReceiveData_flag = 0;                      //串口接收数据标志
uint8_t USART1_REC_BUF[USART1_MAX_RECV_LEN];        //接收数据包缓冲区
uint8_t USART2_REC_BUF_Engine[4];                   //发动机数据包
volatile bool sendDataFlag;																	//是否回应数据标志位
volatile uint8_t	UART1_temp[1];              								//串口1当前接收字节
volatile uint8_t	UART2_temp[1];              								//串口2当前接收字节
volatile uint16_t USART1_REC_STA=0;													//当前字节是连续的第几位
volatile uint16_t USART2_REC_STA=0;													//当前字节是连续的第几位
volatile uint8_t respondDeviceID,respondDeviceValue;					//回应数据的ID和Value
uint16_t RecCrc, CalcCrc;														//接收的CRC校验码和计算的CRC校验码
volatile uint16_t CntRx1 = 15;																		//串口1接收时间标志位,15ms接收不到新数据，判定为1个包
volatile uint16_t CntRx2 = 15;																		//串口2接收时间标志位,15ms接收不到新数据，判定为1个包
//ecu
uint8_t ecuByte0,ecuByte1,ecuByte2;
uint8_t ecuID1_SW = 3;																											//默认紧急停车
uint16_t lastEngineThrottleValue=0, nowEngineThrottleValue=0;					                //上一个发动机的值
uint32_t engineRPM = 0;
uint8_t USART2_Ecu_BUF[USART2_MAX_RECV_LEN];
uint8_t USART2_Ecu_Oil_BUF[USART2_MAX_RECV_LEN];
volatile bool sendEcuDataFlag = false;
//油泵
uint8_t oilByte0,oilByte1,oilByte2;
volatile bool sendOilPumpDataFlag = 0;
uint32_t OilPumpDataCount = 0;
uint8_t EcuOilValue=0, SerialOilValue=0;					                //ecu返回的油泵信息、串口要控制的油泵信息
uint8_t USART2_Oil_BUF[USART1_MAX_RECV_LEN];

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t crc16_calc( uint8_t *, uint8_t);														//CRC校验
void get_rec_data(void);																								//解包函数
void get_ecu_rec_data(void);
void Water_Pump_change(uint8_t Value);
void Powder_Pump_change(uint8_t Value);
void Pump_reset();
void oil_data_send(uint8_t param);
uint8_t crc8_calc(uint8_t *data, uint8_t len);
extern void delay_us(uint16_t);
void TIM_SetCompare3(TIM_TypeDef* TIMx, uint16_t Compare3);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//装载发送数据
void response_data_send(uint8_t deviceID,uint8_t deviceValue)
{
	#if FUNCTION_TEXT
	printf("****************BEGIN LINE********************\r\n");
	
	printf("打印：\n");
	printf("收到的数据为：%x\n",*USART1_REC_BUF);

	printf("****************END LINE**********************\r\n");
	
	#else
	uint8_t data_send[5] = {0};								//串口发送数据
	data_send[0] = 0x0c;
	data_send[1] = deviceID;
	data_send[2] = deviceValue;
	engineRPM = (USART2_REC_BUF_Engine[0] & USART2_REC_BUF_Engine[1])*10;	
	uint16_t resCrc = crc16_calc(data_send, 3);
	data_send[3] = resCrc >> 8;
	data_send[4] = resCrc & 0xff;

	HAL_UART_Transmit(&huart1,data_send,5,1000);

	#endif

}
//装载发送数据
void oil_data_send(uint8_t param)
{
	uint8_t data_send[5] = {0};								//串口发送数据
	data_send[0] = 0xFF;
	data_send[1] = 0x20;
	data_send[2] = param;
	uint16_t resCrc = crc8_calc(data_send, 3);
	data_send[3] = resCrc;

	HAL_UART_Transmit(&huart2,data_send,4,1000);
}
//开机初始化
void Initialize_function(void)
{
	Uart1_ReceiveData_flag = 0;
	WATER_PUMP_OFF;
//	TIM_SetCompare3(TIM1,0);
	SYSTEM_LED_ON;
	POWDER_PUMP_OFF;
}

//解包函数
void get_rec_data(void)
{
	if (USART1_REC_BUF[0] == 0x0C)
	{
		//水泵和粉阀0A、发动机0B、复位0C
		//泵阀    functionID:0x0A  deviceId   水泵：2  粉阀：3
		//发动机  functionID:0x0B  deviceId    油泵：1 发动机紧急停车：0xA0  发动机停车：0xB0 发动机启动：0xC0
		//复位    functionID:0x0C  deviceId   复位：5
		//水泵通讯样例：
		//打开：0C 0A 02 01 D6 E3
		//关闭：0C 0A 02 00 16 22
		//泵阀	0x0A
		if(USART1_REC_BUF[1] == 0x0A)
		{
			RecCrc = ((uint16_t)USART1_REC_BUF[4] << 8) | USART1_REC_BUF[5];
			CalcCrc = crc16_calc(USART1_REC_BUF, 4);
			if(RecCrc != CalcCrc){
				respondDeviceID = 0x00;
				respondDeviceValue = USART1_REC_BUF[3];
				sendDataFlag = true;
				return;
			}
			//控制水泵
			if(USART1_REC_BUF[2] == 0x02)
				Water_Pump_change(USART1_REC_BUF[3]);
			//控制粉阀
			else if(USART1_REC_BUF[2] == 0x03)
				Powder_Pump_change(USART1_REC_BUF[3]);
			else{
				respondDeviceID = 0x00;
				respondDeviceValue = USART1_REC_BUF[3];
			}
			sendDataFlag = true;
		}
		//发动机  functionID:0x0B  deviceId    油泵：1 发动机紧急停车：0xA0  发动机停车：0xB0 发动机启动：0xC0
		//发动机  0x0B
		else if(USART1_REC_BUF[1] == 0x0B)
		{
			RecCrc = ((uint16_t)USART1_REC_BUF[4] << 8) | USART1_REC_BUF[5];
			CalcCrc = crc16_calc(USART1_REC_BUF, 4);
			if(RecCrc != CalcCrc){
				ecuID1_SW = 0x00;
				nowEngineThrottleValue = 0;
			}
			else
			{
				//油泵	0x01
				if(USART1_REC_BUF[2] == 0x01){
					sendOilPumpDataFlag = true;
					OilPumpDataCount = 0;
				}
				//紧急停车	0xA0
				else if(USART1_REC_BUF[2] == 0xA0){
					ecuID1_SW = 0x03;
					nowEngineThrottleValue = 0;
				}
				//停车散热	0xB0
				else if(USART1_REC_BUF[2] == 0xB0){
					ecuID1_SW = 0x02;
					nowEngineThrottleValue = 0;
				}
				//启动	0xC0
				else if(USART1_REC_BUF[2] == 0xC0){
					ecuID1_SW = 0x01;
					nowEngineThrottleValue = (uint16_t)USART1_REC_BUF[3]*10;
				}
				else{
					ecuID1_SW = 0x00;
					nowEngineThrottleValue = 0;
				}
			}		
		}
		//复位	0x0C
		else if(USART1_REC_BUF[1] == 0x0C)
		{
			RecCrc = ((uint16_t)USART1_REC_BUF[4] << 8) | USART1_REC_BUF[5];
			CalcCrc = crc16_calc(USART1_REC_BUF, 4);
			if(RecCrc != CalcCrc){
				respondDeviceID = 0x00;
				respondDeviceValue = USART1_REC_BUF[3];
			}
			else
				Pump_reset();
			sendDataFlag = true;			
		}
		else{
			respondDeviceID = 0x00;
			respondDeviceValue = USART1_REC_BUF[3];
		}
	}
	else{
		respondDeviceID = 0x00;
		respondDeviceValue = USART1_REC_BUF[3];
	}		
}

//控制水泵
void Water_Pump_change(uint8_t Value)
{
	//deviceID:  5：复位   1：油泵   2：水泵   3：粉阀
	if(Value==1){
		WATER_PUMP_ON;
	}
	else if(Value==0){
		WATER_PUMP_OFF;
	}
	else{
		respondDeviceID = 0x00;
		respondDeviceValue = Value;
		return;
	}
	respondDeviceID = 0x02;
	respondDeviceValue = Value;
}
//控制粉阀
void Powder_Pump_change(uint8_t Value)
{
	//deviceID:  5：复位   1：油泵   2：水泵   3：粉阀
	if(Value==1){
		POWDER_PUMP_ON;
	}
	else if(Value==0){
		POWDER_PUMP_OFF;
	}
	else{
		respondDeviceID = 0x00;
		respondDeviceValue = Value;
		return;
	}
	respondDeviceID = 0x03;
	respondDeviceValue = Value;
}
//复位控制
void Pump_reset()
{
	WATER_PUMP_OFF;
	POWDER_PUMP_OFF;
	respondDeviceID = 0x05;
	respondDeviceValue = 1;
}
//16位CRC校验
uint16_t crc16_calc(uint8_t *crc_buf , uint8_t crc_leni)
{
	unsigned char i, j;
	unsigned int crc_sumx;

	crc_sumx = 0xFFFF;

	for (i = 0; i < crc_leni; i++) 
	{
		crc_sumx ^= *(crc_buf + i);
		for (j = 0; j < 8; j++) {
			if (crc_sumx & 0x01) {
				crc_sumx >>= 1;
				crc_sumx ^= 0xA001;
			} else {
				crc_sumx >>= 1;
			}
		}
	}
	return crc_sumx;
}
//8位CRC校验
uint8_t crc8_calc(uint8_t *data, uint8_t len){
    uint8_t crc8 = 0x00;
    for (uint8_t byte=0; byte<len;byte++) {
        crc8 ^= data[byte];
        for (int i = 0; i < 8; ++i) {
            if (crc8 & 0x80) {
                crc8 = (uint8_t)((crc8 << 1) ^ 0x31);
            } else {
                crc8 <<= 1;
            }
        }
    }
    return crc8;
}
/**************************************多任务调度*******************************************/
//回答上位机请求任务
void Task_USART1_Respond(void)
{
	if(Uart1_ReceiveData_flag){
		__disable_irq(); // 进入临界区

		get_rec_data();
		
		//泵阀
		if(sendDataFlag == true){
			response_data_send(respondDeviceID,respondDeviceValue);
			sendDataFlag = false;
		}
		
		memset(USART1_REC_BUF, 0, sizeof (USART1_REC_BUF));
		
		USART1_REC_STA = 0;
		Uart1_ReceiveData_flag = 0;
		__enable_irq();; // 退出临界区
		
	}

}
//发动机油泵控制任务
void Task_OilPump_Control(void)
{
	//油泵
	if(sendOilPumpDataFlag == true){
		if(OilPumpDataCount <= 1300){
			//串口2发送油泵数据
			oil_data_send(0x01);
		}
		else{
			sendOilPumpDataFlag = false;
			OilPumpDataCount = 0;
			oil_data_send(0x0D);
		}
	}

}
//发动机油门控制任务
void Task_Engine_Throttle_Control(void)
{
	ecuByte0 = 0xFF;
	ecuByte1 = (1<<4)+(ecuID1_SW<<2)+(((uint8_t)(nowEngineThrottleValue>>8))&0x0F);
	ecuByte2 = nowEngineThrottleValue & 0xFF;
	USART2_Ecu_BUF[0] = ecuByte0;
	USART2_Ecu_BUF[1] = ecuByte1;
	USART2_Ecu_BUF[2] = ecuByte2;
	USART2_Ecu_BUF[3] = crc8_calc(USART2_Ecu_BUF,3);
	lastEngineThrottleValue = nowEngineThrottleValue;
	HAL_UART_Transmit(&huart2,USART2_Ecu_BUF,4,1000);
}
//ECU解包任务
void Task_get_ecu_rec_data(void)
{
//	printf("Task_get_ecu_rec_data\n");
}

//全部任务列表
static TASK_COMPONENTS TaskComps[] = 
{
	{0, 10, 10, Task_USART1_Respond},																//串口1接收数据并处理
	{0, 20, 20, Task_OilPump_Control},															//发动机油泵控制任务
	{0, 20, 20, Task_Engine_Throttle_Control},											//串口2向发动机定时发送油门数据
	{0, 10, 10, Task_get_ecu_rec_data}															//ECU解包任务
   // 这里添加你的任务。。。
};
/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : 任务标志处理，这个函数要放在定时器里
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskRemarks(void)
{
    uint8_t i;
    for (i=0; i<TASKS_MAX; i++)          // 逐个任务时间处理
    {
         if (TaskComps[i].Timer)          // 时间不为0
        {
            TaskComps[i].Timer--;         // 减去一个节拍
            if (TaskComps[i].Timer == 0)       // 时间减完了
            {
                 TaskComps[i].Timer = TaskComps[i].ItvTime;       // 恢复计时器值，从新下一次
                 TaskComps[i].Run = 1;           // 任务可以运行
            }
        }
    }
}
/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : 任务处理，放到main函数里去执行的。
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskProcess(void)
{
    uint8_t i;
    for (i=0; i<TASKS_MAX; i++)               // 逐个任务时间处理
    {
         if (TaskComps[i].Run)                // 时间不为0
        {
             TaskComps[i].TaskHook();         // 运行任务
             TaskComps[i].Run = 0;            // 标志清0
        }
    }   
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//系统初始化
	Initialize_function();
	
	//启动串口接收
	HAL_UART_Receive_IT(&huart1, (uint8_t *)UART1_temp, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)UART2_temp, 1);
	//开启定时器中断----->1ms
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_DBGMCU_EnableDBGStopMode();
	//开启PWM
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
		TaskProcess();

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
