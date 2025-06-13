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
#include "dma.h"
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
volatile uint8_t Uart1_ReceiveData_flag = 0;                      //串口1接收数据标志
uint8_t USART1_REC_BUF[USART1_MAX_RECV_LEN];        //接收数据包缓冲区
uint8_t USART2_REC_BUF_Engine[USART1_MAX_RECV_LEN];                   //发动机数据包
volatile bool sendDataFlag;																	//是否回应数据标志位
volatile uint8_t	UART1_temp[1];              								//串口1当前接收字节
volatile uint16_t USART1_REC_STA=0;													//当前字节是连续的第几位
volatile uint8_t respondDeviceID,respondDeviceValue;					//回应数据的ID和Value
uint16_t RecCrc, CalcCrc;														//接收的CRC校验码和计算的CRC校验码
uint8_t ecuCalcCrc = 0;
volatile uint16_t CntRx1 = 0;																		//串口1接收时间标志位,15ms接收不到新数据，判定为1个包

//ecu
bool ecuValueSendFlag = false;
uint8_t ecuByte0,ecuByte1,ecuByte2;
uint8_t ecuID1_SW = 0;																											//默认不控制
uint8_t ecuThrottleControl = 0;
uint16_t lastEngineThrottleValue=0, nowEngineThrottleValue=0;					                //上一个发动机的值
volatile uint8_t Uart2_ReceiveData_flag = 0;                      //串口2接收数据标志
uint8_t USART2_Ecu_BUF[USART2_MAX_RECV_LEN];
volatile uint16_t CntRx2 = 0;																		//串口2接收时间标志位,15ms接收不到新数据，判定为1个包
volatile uint16_t USART2_REC_STA=0;													//当前字节是连续的第几位
volatile uint8_t	UART2_temp[1];              								//串口2当前接收字节
uint8_t USART2_Ecu_Oil_BUF[USART2_MAX_RECV_LEN];
volatile bool sendEcuDataFlag = false;
//ecu参数
uint8_t ecuParaByteSend[13]={0};
uint16_t engineRPM = 0;
uint8_t powerVoltage = 0;
uint16_t current = 0;
uint8_t pumpVoltage = 0;
uint16_t temp = 0;
uint8_t ECode = 0;
uint8_t engineState = 0;

//油泵
uint8_t oilByte0,oilByte1,oilByte2;
volatile bool sendOilPumpDataFlag = 0;
uint32_t OilPumpDataCount = 0;
uint8_t EcuOilValue=0, SerialOilValue=0;					                //ecu返回的油泵信息、串口要控制的油泵信息
uint8_t USART2_Oil_BUF[USART1_MAX_RECV_LEN];

const uint8_t crc_array[] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
	0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
	0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
	0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
	0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
	0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
	0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
	0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
	0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
	0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
	0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
	0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
	0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
	0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
	0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
	0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
	0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};


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
uint8_t crc8_calc(uint8_t *data, uint8_t len);
uint8_t ecu_crc8(uint8_t *puchMsg,uint8_t crc_len, uint8_t seed);
uint8_t crc8_ecu(uint8_t *puchMsg,uint8_t crc_len, uint8_t seed);
void get_rec_data(void);																								//解包函数
void get_ecu_rec_data();
void Water_Pump_change(uint8_t Value);
void Powder_Pump_change(uint8_t Value);
void Pump_reset();
void oil_data_send(uint8_t param);
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
	uint8_t data_send[7] = {0};								//串口发送数据
	data_send[0] = 0x0C;
	data_send[1] = deviceID;
	data_send[2] = deviceValue;
	data_send[3] = 0x00;
	data_send[4] = 0x00;
	uint16_t resCrc = crc16_calc(data_send, 5);
	data_send[5] = resCrc >> 8;
	data_send[6] = resCrc & 0xff;

	HAL_UART_Transmit(&huart1,data_send,7,1000);

	#endif

}
//装载发送数据
void oil_data_send(uint8_t param)
{
	uint8_t USART2_Ecu_Crc[2] = {0};
	USART2_Ecu_Crc[0] = 0x20;
	USART2_Ecu_Crc[1] = param;
	
	uint8_t data_send[5] = {0};								//串口发送数据
	data_send[0] = 0xFF;
	data_send[1] = 0x20;
	data_send[2] = param;
	uint8_t resCrc = crc8_ecu(USART2_Ecu_Crc,2,0);
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
		//发动机  functionID:0x0B  deviceId    油泵：1 发动机紧急停车：0xA0  发动机停车：0xB0 发动机启动：0xC0 开启控制：0xD0
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
		//发动机  functionID:0x0B  deviceId    油泵：1 发动机紧急停车：0xA0  发动机停车：0xB0 发动机启动：0xC0 开启控制：0xD0
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
//					OilPumpDataCount = 0;
				}
				//紧急停车	0xA0
				else if(USART1_REC_BUF[2] == 0xA0){
					ecuID1_SW = 0x01;
					nowEngineThrottleValue = 0;
				}
				//停车散热	0xB0
				else if(USART1_REC_BUF[2] == 0xB0){
					ecuID1_SW = 0x02;
					nowEngineThrottleValue = 0;
				}
				//启动	0xC0
				else if(USART1_REC_BUF[2] == 0xC0){
					ecuID1_SW = 0x03;
					nowEngineThrottleValue = (uint16_t)USART1_REC_BUF[3]*10;
				}
				//开启控制 0xC0
				else if(USART1_REC_BUF[2] == 0xD0){
					if(USART1_REC_BUF[3]==0)
						ecuThrottleControl = 0;
					else{
						ecuThrottleControl = 1;
						ecuID1_SW = 0x01;
					}
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

//解包函数
void get_ecu_rec_data()
{
//	实例 1：0xF1 0x00 0x00 0x00 0x00 0x4C 0xA0
//	F1 00 00 00 00 4C A0
//	ID: 1
//	RPM: 0 引擎转速
//	Engine State: 0 引擎状态 (停机)
//	ECode: 0 错误代码
//	Temp: 26℃ 引擎排气温度（摄氏度）
//	SwSt: 0 主机(计算机、飞控)给 ECU 的控制状态(引擎停机)
//	-------------------------------------------------
//	实例 2：0xF6 0x00 0x00 0xA0 0x00 0x10 0xAB
//	F6 00 00 A0 00 10 AB
//	ID: 6
//	RPM: 0 引擎转速
//	Max RPM: 160000 引擎最大转速
//	Max Pump Voltage: 0.0V 已经学习的油泵最高电压
//	Protocol Version: 4 协议版本
//	SRate: 20Hz 当前数据更新速率
	uint8_t head,cmd_id;
	head = USART2_REC_BUF_Engine[0] & 0xF0;
	cmd_id = USART2_REC_BUF_Engine[0] & 0x0F;
	if(head == 0xF0)
	{
		ecuCalcCrc = crc8_ecu(USART2_REC_BUF_Engine,3,0x03);
		if(ecuCalcCrc != USART2_REC_BUF_Engine[6]){
			return;
		}
		else{
			if(cmd_id == 0x01)
			{
				engineRPM = (USART2_REC_BUF_Engine[2] << 8) + USART2_REC_BUF_Engine[1];
				engineState = USART2_REC_BUF_Engine[3] & 0x0F;
				temp = (((uint16_t)(USART2_REC_BUF_Engine[4] & 0x0E)) << 8) + (uint16_t)(USART2_REC_BUF_Engine[5]);
					
			}
			else if(cmd_id == 0x02)
			{
				engineRPM = (USART2_REC_BUF_Engine[2] << 8) + USART2_REC_BUF_Engine[1];
				powerVoltage = USART2_REC_BUF_Engine[4];
			}
			else if(cmd_id == 0x04)
			{
				engineRPM = (USART2_REC_BUF_Engine[2] << 8) + USART2_REC_BUF_Engine[1];
				current = ((USART2_REC_BUF_Engine[4] & 0x01) << 8) + USART2_REC_BUF_Engine[3];
			}
			else{
				engineRPM = (USART2_REC_BUF_Engine[2] << 8) + USART2_REC_BUF_Engine[1];
			}
		}		
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
uint8_t crc8_ecu(uint8_t *puchMsg,uint8_t crc_len, uint8_t seed)
{
	uint8_t i,k,crc8 = seed;
	for(i = 0; i < crc_len; i++)
	{
		k = puchMsg[i] ^ crc8;
		crc8 = 0;
		if (k & 0x01) crc8 ^= 0x5e;
		if (k & 0x02) crc8 ^= 0xbc;
		if (k & 0x04) crc8 ^= 0x61;
		if (k & 0x08) crc8 ^= 0xc2;
		if (k & 0x10) crc8 ^= 0x9d;
		if (k & 0x20) crc8 ^= 0x23;
		if (k & 0x40) crc8 ^= 0x46;
		if (k & 0x80) crc8 ^= 0x8c;
	}
	k = 0;
	return crc8;
}
uint8_t ecu_crc8(uint8_t *puchMsg,uint8_t crc_len, uint8_t seed)
{
	uint8_t i,ecuCrc8 = seed;
	for(i = 0 ; i < crc_len; i++)
	{
		ecuCrc8 = crc_array[ecuCrc8^puchMsg[i]];
	}
	return ecuCrc8;
}
//串口2空闲中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART2)
	{
//		uint32_t tmp_flag ;
//		tmp_flag  = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE);

//		if( tmp_flag  != RESET)
//		{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
		HAL_UART_DMAStop(&huart2); //停止DMA接收，防止数据出错
		//单片机解包方式		
//		get_ecu_rec_data();
//		ecuParaByteSend[0] = 0x1F;
//		ecuParaByteSend[1] = engineRPM & 0xFF;
//		ecuParaByteSend[2] = engineRPM >> 8;
//		ecuParaByteSend[3] = powerVoltage;
//		ecuParaByteSend[4] = current & 0xFF;
//		ecuParaByteSend[5] = current >> 8;
//		ecuParaByteSend[6] = pumpVoltage;
//		ecuParaByteSend[7] = temp & 0xFF;
//		ecuParaByteSend[8] = temp >> 8;
//		ecuParaByteSend[9] = ECode;
//		ecuParaByteSend[10] = engineState;
//		uint16_t resCrc = crc16_calc(ecuParaByteSend, 11);
//		ecuParaByteSend[11] = resCrc >> 8;
//		ecuParaByteSend[12] = resCrc & 0xff;
//		HAL_UART_Transmit(&huart1,ecuParaByteSend,13,1000);
//		USART2_REC_STA = 0;
//		if(Uart2_ReceiveData_flag)
//			Uart2_ReceiveData_flag = 0;
		//单片机转发方式
		//改为标志位置1
		ecuValueSendFlag = true;
//		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,USART2_REC_BUF_Engine,USART1_MAX_RECV_LEN);		// 再次开启DMA空闲中断
	}
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
		oil_data_send(0x0C);
		sendOilPumpDataFlag = false;
//		if(OilPumpDataCount <= 1300){
//			//串口2发送油泵数据
//			oil_data_send(0x0C);
//		}
//		else{
//			sendOilPumpDataFlag = false;
//			OilPumpDataCount = 0;
//			oil_data_send(0x0D);
//		}
	}

}
//发动机油门控制任务
void Task_Engine_Throttle_Control(void)
{
	__disable_irq(); // 进入临界区
	if(ecuThrottleControl == 0){
		ecuID1_SW = 0;
		nowEngineThrottleValue = 0;
	}
	uint8_t USART2_Ecu_Crc[2] = {0};
	ecuByte0 = 0xFF;
	ecuByte1 = (1<<4)+(ecuID1_SW<<2)+(((uint8_t)(nowEngineThrottleValue>>8))&0x0F);
	ecuByte2 = nowEngineThrottleValue & 0xFF;
	USART2_Ecu_Crc[0] = ecuByte1;
	USART2_Ecu_Crc[1] = ecuByte2;
	
	USART2_Ecu_BUF[0] = ecuByte0;
	USART2_Ecu_BUF[1] = ecuByte1;
	USART2_Ecu_BUF[2] = ecuByte2;
	USART2_Ecu_BUF[3] = crc8_ecu(USART2_Ecu_Crc,2,0);
	lastEngineThrottleValue = nowEngineThrottleValue;
	HAL_UART_Transmit_DMA(&huart2,USART2_Ecu_BUF,4);
	__enable_irq();; // 退出临界区
}
//串口1向上位机定时发送ECU参数数据任务
void Task_ECU_getvalue_Send(void)
{
	if(ecuValueSendFlag)
	{
		HAL_UART_Transmit(&huart1,USART2_REC_BUF_Engine,7,1000);
		memset(USART2_REC_BUF_Engine, 0, sizeof (USART2_REC_BUF_Engine));
		ecuValueSendFlag = false;
	}
}

//全部任务列表
static TASK_COMPONENTS TaskComps[] = 
{
	{0, 10, 10, Task_USART1_Respond},																//串口1接收数据并处理
	{0, 20, 20, Task_OilPump_Control},															//发动机油泵控制任务
	{0, 20, 20, Task_Engine_Throttle_Control},											//串口2向发动机定时发送油门数据
	{0, 10, 10, Task_ECU_getvalue_Send}															//串口1向上位机定时发送ECU参数数据
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//系统初始化
	Initialize_function();
	
	//启动串口接收
	HAL_UART_Receive_IT(&huart1, (uint8_t *)UART1_temp, 1);
//	HAL_UART_Receive_IT(&huart2, (uint8_t *)UART2_temp, 1);
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); //使能IDLE中断
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,USART2_REC_BUF_Engine,USART1_MAX_RECV_LEN);		// 再次开启DMA空闲中断
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
