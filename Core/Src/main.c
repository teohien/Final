/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
  uint8_t Frequency[2];
  uint8_t Threshold[3];
}myUartQueueData_t;

typedef struct{
	uint8_t temp;
  uint8_t humi;
  uint8_t distance;
}myQueueData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TransmitBuff_SIZE	35
#define ReceiveBuff_SIZE	10
#define MainBuff_SIZE		8
#define Queue_SIZE			3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId DHTTaskHandle;
osThreadId HCSR04TaskHandle;
osThreadId LCDTaskHandle;
osThreadId UARTTaskHandle;
osThreadId ButtonTaskHandle;
osThreadId ThresholdTaskHandle;

osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;

/* Data of DHT11 Sensor */
float temp = 0;
float humi = 0;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;
uint8_t Presence = 0;

/* Data line of LCD1602*/
char data_line_1[16];
char data_line_2[16];

/* Data of HC-SR04 Sensor */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

/* Buffer Array */
uint8_t RxBuffer[ReceiveBuff_SIZE];
uint8_t TxBuffer[TransmitBuff_SIZE];
uint8_t MainBuffer[MainBuff_SIZE];

/* Period Time */
uint8_t DHT11_transmit_period;
uint8_t HCSR04_transmit_period;

/* Temperature Threshold */
uint8_t temp_threshold;

/* Humidity Threshold */
uint8_t humi_threshold;

/* Distance Threshold */
uint8_t dist_threshold;


/* Etc .. */
uint8_t cmd_index = 1;
uint8_t len = 0;

uint32_t start_tick = 0;
uint32_t stop_tick = 0;
uint32_t execute_tick = 0;

bool read_sensor_flag = 0;

uint16_t Count = 0;
uint16_t cnt = 0;
uint16_t cnt_1 = 0;
uint8_t isr_cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void DHTTask(void *Param);
void HCSR04Task(void *Param);
void LCDTask(void *Param);
void UARTTask(void *Param);
void ButtonTask(void *Param);
void ThresholdTask(void *Param);
uint8_t Rx_data[2];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
 
  /* USER CODE BEGIN RTOS_THREADS */

  osMailQDef(myQueue01, 14, myQueueData_t);
  myQueue01Handle = osMailCreate(osMailQ(myQueue01), NULL);

  osMailQDef(myQueue02, 14, myUartQueueData_t);
  myQueue02Handle = osMailCreate(osMailQ(myQueue02), NULL);

  osThreadDef(DHTTask, osPriorityNormal, 0, 128*4);
  DHTTaskHandle  = osThreadCreate(DHTTask, NULL);
  
  osThreadDef(HCSR04Task, osPriorityBelowNormal, 0, 128*4);
  HCSR04TaskHandle  = osThreadCreate(HCSR04Task, NULL);

  osThreadDef(LCDTask, osPriorityAboveNormal, 0, 128*4);
  LCDTaskHandle  = osThreadCreate(DHTTask, NULL);

  osThreadDef(UARTTask, osPriorityAboveNormal, 0, 128*4);
  UARTTaskHandle  = osThreadCreate(UARTTask, NULL);

  osThreadDef(ButtonTask, osPriorityAboveNormal, 0, 128*4);
  ButtonTaskHandle  = osThreadCreate(ButtonTask, NULL);

  osThreadDef(ThresholdTask, osPriorityAboveNormal, 0, 128*4);
  ThresholdTaskHandle  = osThreadCreate(ThresholdTask, NULL);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Xử lý ngắt nút nhấn
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriority;
  xHigherPriority = xTaskResumeFromISR(ButtonTask);
  portEND_SWITCHING_ISR(xHigherPriority);
}
// Xử lý ngắt  UART, nhảy vào hàm 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART2){
		for(size_t i = 0; i<Size-1; i++) {
			if(RxBuffer[i] == 0x00){
				memcpy(MainBuffer, &RxBuffer[i], Size-i);
				processRecCmd(Size-i);
				break;
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuffer, MainBuff_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  // Gọi Task UART để xử lý
  BaseType_t xHigher = xTaskResumeFromISR(UARTTask);
  portEND_SWITCHING_ISR(xHigher);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart2, &Rx_data, 1);
  BaseType_t xHigher = xTaskResumeFromISR(UARTTask);
  portEND_SWITCHING_ISR(xHigher);
}
// Task đọc và gửi nhiệt độ
void DHTTask(void *Param)
{
  myQueueData_t *msg = osMailAlloc(myQueue01Handle, osWaitForever);
  while(1)
  {

// Đưa nhiệt độ, độ ẩm vào queue01 để hiển thị LCD'
   msg->temp = temp;
   msg->humi = humi;
   osMailPut(myQueue01Handle, msg);
  }
}
// Task đọc và gửi khoảng cách
void HCSR04Task(void *Param)
{
  myQueueData_t *msg = osMailAlloc(myQueue01Handle, osWaitForever);
  while(1)
  {
    msg->distance = Distance;
    osMailPut(myQueue01Handle, msg);
  }
}
// Task hiển thị LCD
void LCDTask(void *Param)
{
  osEvent Recv_Task_data; // nhận dữ liệu từ Queue
  while(1)
  {
    Recv_Task_data = osMailGet(myQueue01Handle, osWaitForever);
    myQueueData_t *data = Recv_Task_data.value.p; // con trỏ đến cấu trúc lưu nhiệt độ, độ ẩm
    // Xử lý hiển thị

    // nhận xong phải giải phóng
    osMailFree(myQueue01Handle, data);
  }
}
// Task xử lý khi nhân được ngắt UART
// Nhận dữ liệu và cập nhật ngưỡng vào Queue2 để các task khác xử lý
// Thêm 1 task ngưỡng, dùng suspend và resume
void UARTTask(void *Param)
{
  while(1)
  {
    vTaskSuspend(NULL);
    myQueueData_t *msg = osMailAlloc(myQueue01Handle, osWaitForever);
    // Đưa các biến Command; Frequency; Threshold vào Queue2 để các task khác xử lý

    
  }
}
//Task xử lý khi nhấn nút
void ButtonTask(void *Param)
{
  while(1)
  {
    vTaskSuspend(NULL);
    printf("hello from AboveNormal Task\n");
  }
}

//Task xử lý ngưỡng
void ThresholdTask(void *Param)
{
  while(1)
  {
    vTaskSuspend(NULL);
    printf("hello from AboveNormal Task\n");
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
