/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RF24.h"
#include <string.h>
#include "meteo_home.h"
#include "receiver.h"
#include <math.h>
#include <stdio.h>
#include <task.h>

#include "receiver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SERIAL_DEBUG 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint64_t pipe_addresses[6] = {
	0xF0F0F0F0D2LL,
	0xF0F0F0F0E1LL,
	0xF0F0F0F0E2LL,
	0xF0F0F0F0E3LL,
	0xF0F0F0F0E4LL,
	0xF0F0F0F0E5LL
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myRadioHandle;
osThreadId unixTimeCounterHandle;
osMessageQId msgUnixTimerHandle;
osMessageQId msgPipeAllocHandle;
osTimerId osUnixTimerHandle;
osMutexId unixtimeMutexHandle;
/* USER CODE BEGIN PV */
osPoolDef(pipe_pool, 6, struct ReceivedData);
osPoolId  pipe_pool_id;

osTimerId osUnixTimer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartRadio(void const * argument);
void StartUnixTimeCounter(void const * argument);
void CallbackUnixTimer(void const * argument);

/* USER CODE BEGIN PFP */
TickType_t xLastWakeTime,xAfterSent;
const TickType_t xFrequency = 1000;

osEvent event1;

//volatile uint32_t time_counter=0;
//volatile uint32_t _unixtime=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile   uint32_t _unixtime;
char str_rx[21];
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init(); // счётчик для микросекундных пауз
  pipe_pool_id = osPoolCreate(osPool(pipe_pool));
  //MX_USB_DEVICE_Init();
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of unixtimeMutex */
  osMutexDef(unixtimeMutex);
  unixtimeMutexHandle = osMutexCreate(osMutex(unixtimeMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of osUnixTimer */
  osTimerDef(osUnixTimer, CallbackUnixTimer);
  osUnixTimerHandle = osTimerCreate(osTimer(osUnixTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of msgUnixTimer */
  osMessageQDef(msgUnixTimer, 16, uint32_t);
  msgUnixTimerHandle = osMessageCreate(osMessageQ(msgUnixTimer), NULL);

  /* definition and creation of msgPipeAlloc */
  osMessageQDef(msgPipeAlloc, 16, uint32_t);
  msgPipeAllocHandle = osMessageCreate(osMessageQ(msgPipeAlloc), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myRadio */
  osThreadDef(myRadio, StartRadio, osPriorityAboveNormal, 0, 128);
  myRadioHandle = osThreadCreate(osThread(myRadio), NULL);

  /* definition and creation of unixTimeCounter */
  osThreadDef(unixTimeCounter, StartUnixTimeCounter, osPriorityNormal, 0, 128);
  unixTimeCounterHandle = osThreadCreate(osThread(unixTimeCounter), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IND_GPIO_Port, IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IND_Pin */
  GPIO_InitStruct.Pin = IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF24_IRQ_Pin */
  GPIO_InitStruct.Pin = RF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF24_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
	  static uint8_t rx_ready;
	  static uint8_t status;
	  static uint8_t pipeNo;
	  status = whatHappened();
	  rx_ready = status & (1 << RX_DR);
	  if (rx_ready){
		  while( available(&pipeNo)){              // Read all available payloads
			struct ReceivedData *pPipeData  = osPoolAlloc(pipe_pool_id);
			pPipeData->pipeNo = pipeNo;
			status = getPayloadSize();
			read( &pPipeData->data, sizeof(struct meteo_data_struct));

			osMessagePut(msgPipeAllocHandle, (uint32_t)pPipeData, 0);
			}
		}
  }
  else
    {
      __NOP();

    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOC,IND_Pin);
    osDelay(500);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartRadio */

/* USER CODE END Header_StartRadio */
void StartRadio(void const * argument)
{
  /* USER CODE BEGIN StartRadio */

	  receiver_init();
	  uint8_t res = isChipConnected(); // проверяет подключён ли модуль к SPI

	  static char str[100] = {0,};
	  snprintf(str, 64, "Connected: %s\n", res ? "OK" : "NOT OK");
	  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

	  res = NRF_Init(); // инициализация
#if defined( SERIAL_DEBUG)
	  snprintf(str, 64, "Init: %s\n", res > 0 && res < 255 ? "OK" : "NOT OK");
	  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
#endif
	  ////////////// SET ////////////////

	  setPALevel(RF24_PA_LOW);
	  //uint8_t status = getPALevel();
	  //setAutoAck(false);
	  setPayloadSize(sizeof(struct meteo_data_struct));
	  enableDynamicPayloads();
	  enableAckPayload();
	  setChannel(106);
		  openWritingPipe(pipe_addresses[0]);
		  openReadingPipe(1,pipe_addresses[1]); //0xF0F0F0F0E1LL);
		  openReadingPipe(2,pipe_addresses[2]);
		  openReadingPipe(3,pipe_addresses[3]);
		  openReadingPipe(4,pipe_addresses[4]);
		  openReadingPipe(5,pipe_addresses[5]);
		  CreateNullAck(1);
		  writeAckPayload(1,&pipeData[1].ackData, sizeof(struct server_ack ));
		  CreateNullAck(2);
		  writeAckPayload(2,&pipeData[2].ackData, sizeof(struct server_ack ));
		  CreateNullAck(3);
		  writeAckPayload(3,&pipeData[3].ackData, sizeof(struct server_ack ));
		  CreateNullAck(4);
		  writeAckPayload(4,&pipeData[4].ackData, sizeof(struct server_ack ));
		  CreateNullAck(5);
		  writeAckPayload(5,&pipeData[5].ackData, sizeof(struct server_ack ));
		  //writeAckPayload(3,&pipeData[3].ackData, sizeof(struct server_ack ));
		  startListening();


	  static osEvent event;

  /* Infinite loop */
  for(;;)
  {
	  	//static uint8_t pipeLine;
	  	static char strT[8];

	  		event = osMessageGet(msgPipeAllocHandle, osWaitForever);
	  		if(event.status == osEventMessage)
	  		{
	  			struct ReceivedData *pPipeData = event.value.p;
	  			PackDataToAck(pPipeData);
				writeAckPayload(pPipeData->pipeNo,&pipeData[pPipeData->pipeNo].ackData, sizeof(struct server_ack ));
				startListening();

#if defined( SERIAL_DEBUG)
	  			unixtimeToString( pPipeData->data.meteo_data.measurement_time,(char*)&strT);

	  			snprintf(str, 100, "\r\npipe %i got: T=%i P=%i H=%i state=%i power=%i q=%i type=%i delay=%lius  time= %s Vcc=%i\r\n",
	  			pPipeData->pipeNo,
	  			pPipeData->data.meteo_data.T,
				pPipeData->data.meteo_data.P,
				pPipeData->data.meteo_data.H,
				pPipeData->data.state,
				pPipeData->data.power,
				pPipeData->data.query,
				pPipeData->data.type_of_data,
				pPipeData->data.round_tripDelay,
				  //pPipeData->data.unixtime,
				strT,
				pPipeData->data.vcc);
#endif
	  			//ShowMessage(str);
    			osPoolFree(pipe_pool_id,pPipeData);
#if defined( SERIAL_DEBUG)
	  			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
    			  unixtimeToString(unixtime,(char*)&strT);
  	  			snprintf(str, 100, "now: time= %s\r\n",strT);
	  			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
	  			osDelay(100);
#endif
	  		}
  }
  /* USER CODE END StartRadio */
}

/* USER CODE BEGIN Header_StartUnixTimeCounter */
/**
* @brief Function implementing the unixTimeCounter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUnixTimeCounter */
void StartUnixTimeCounter(void const * argument)
{
  /* USER CODE BEGIN StartUnixTimeCounter */
	osTimerStart(osUnixTimerHandle, 1000);
	  static osEvent event;
	  static uint8_t strT[8];

	  unixtime = 946681200;
	    unixtimeToString(unixtime,(char*)&strT);
#if defined( SERIAL_DEBUG)
		  static char str[100] = {0,};
		  sprintf(str, "start time: %s unixtime=%li\r\n",strT,unixtime);
		  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
#endif


  /* Infinite loop */
  for(;;)
  {
	  event = osMessageGet(msgUnixTimerHandle, osWaitForever);
	  if(event.status == osEventMessage)
      {

		  xSemaphoreTake(unixtimeMutexHandle, portMAX_DELAY);
		  unixtime++;
		  xSemaphoreGive(unixtimeMutexHandle);
		  if (unixtime % 60 ==0)
		  {
		    unixtimeToString(unixtime,(char*)&strT);
#if defined( SERIAL_DEBUG)
		    snprintf(str,100, "\r\nserver time: %s unixtime=%li  %i\r\n\r\n",strT,unixtime,event.value.v);
		    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
#endif

		  }

      }
  }
  /* USER CODE END StartUnixTimeCounter */
}

/* CallbackUnixTimer function */
void CallbackUnixTimer(void const * argument)
{
  /* USER CODE BEGIN CallbackUnixTimer */
  static uint32_t tim = 0;
  //tim++;

  osMessagePut(msgUnixTimerHandle, tim, 0);
  /*
  xSemaphoreTake(unixtimeMutexHandle, portMAX_DELAY);
  _unixtime = tim;
  xSemaphoreGive(unixtimeMutexHandle);
  */
  /* USER CODE END CallbackUnixTimer */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2) {
    relayDelay++;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
