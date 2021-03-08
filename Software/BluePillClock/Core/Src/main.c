/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Queue.h"
#include "usbd_cdc_if.h"
#include "time.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    QueueHandle_t queueTime ;
    QueueHandle_t queueDsp  ;
} Queue_readTime_t ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* Definitions for rtcRead_Task */
osThreadId_t rtcRead_Task_Handle;
const osThreadAttr_t rtcRead_Task_attributes = {
  .name = "rtcRead_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for dspTask */
osThreadId_t dspTask_Handle;
const osThreadAttr_t dspTask_attributes = {
  .name = "dspTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

void rtcRead_Task( void * argument) ;
void dspTask( void * argument) ;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    Queue_readTime_t Queue_readTime ;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  Queue_readTime.queueTime = xQueueCreate( 8 , sizeof( time_t ) );
  Queue_readTime.queueDsp  = xQueueCreate( 8 , sizeof( uint8_t ) * 14 ) ;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, ( void * ) Queue_readTime.queueTime , &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  rtcRead_Task_Handle = osThreadNew(rtcRead_Task , ( void * ) &Queue_readTime , &rtcRead_Task_attributes);
  dspTask_Handle      = osThreadNew(dspTask      , ( void * ) Queue_readTime.queueDsp  , &dspTask_attributes     );
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BP_06_Pin|BP_10_Pin|BP_05_Pin|BP_01_Pin
                          |BP_00_Pin|BP_04_Pin|BP_09_Pin|BP_08_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BP_B_Pin|BP_F_Pin|BP_G_Pin|BP_A_Pin
                          |BP_E_Pin|BP_D_Pin|BP_C_Pin|BP_P_Pin
                          |BP_11_Pin|BP_02_Pin|BP_07_Pin|BP_12_Pin
                          |BP_03_Pin|BP_13_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BP_LED_Pin */
  GPIO_InitStruct.Pin = BP_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BP_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BP_06_Pin BP_10_Pin BP_05_Pin BP_01_Pin
                           BP_00_Pin BP_04_Pin BP_09_Pin BP_08_Pin */
  GPIO_InitStruct.Pin = BP_06_Pin|BP_10_Pin|BP_05_Pin|BP_01_Pin
                          |BP_00_Pin|BP_04_Pin|BP_09_Pin|BP_08_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BP_B_Pin BP_F_Pin BP_G_Pin BP_A_Pin
                           BP_E_Pin BP_D_Pin BP_C_Pin BP_P_Pin
                           BP_11_Pin BP_02_Pin BP_07_Pin BP_12_Pin
                           BP_03_Pin BP_13_Pin */
  GPIO_InitStruct.Pin = BP_B_Pin|BP_F_Pin|BP_G_Pin|BP_A_Pin
                          |BP_E_Pin|BP_D_Pin|BP_C_Pin|BP_P_Pin
                          |BP_11_Pin|BP_02_Pin|BP_07_Pin|BP_12_Pin
                          |BP_03_Pin|BP_13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BP_BOOT1_Pin BP_NOTUSED_0_Pin */
  GPIO_InitStruct.Pin = BP_BOOT1_Pin|BP_NOTUSED_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BP_SW1_Pin BP_SW2_Pin */
  GPIO_InitStruct.Pin = BP_SW1_Pin|BP_SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BP_NOTUSED_2_Pin BP_NOTUSED_1_Pin */
  GPIO_InitStruct.Pin = BP_NOTUSED_2_Pin|BP_NOTUSED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void rtcRead_Task( void * argument)
{
    HAL_StatusTypeDef s_retHal ;
    int s32_ret ;

    RTC_TimeTypeDef s_timePrevious ;
    RTC_TimeTypeDef s_timeCurrent  ;

    Queue_readTime_t * p_Queue_readTime ;
    time_t epoch = 0 ;

    p_Queue_readTime = ( Queue_readTime_t * ) argument ;

    ( void ) memset( &s_timePrevious , 0x00 , sizeof( s_timePrevious ) ) ;
    ( void ) memset( &s_timeCurrent  , 0x00 , sizeof( s_timeCurrent  ) ) ;

    for( ; /* EVER */ ; )
    {
        s_retHal = HAL_RTC_GetTime( &hrtc , &s_timeCurrent , RTC_FORMAT_BIN ) ;
        if( HAL_OK != s_retHal )
        {
            continue ;
        }

        s32_ret = memcmp( &s_timeCurrent , &s_timePrevious , sizeof( s_timeCurrent ) ) ;
        if( 0 != s32_ret )
        {
            ( void ) memcpy( &s_timePrevious , &s_timeCurrent , sizeof( s_timePrevious ) ) ;
            HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_RESET);

            epoch++ ;

            xQueueSend( p_Queue_readTime->queueTime , &epoch, 100 );
        }
        else
        {
            HAL_GPIO_WritePin(BP_LED_GPIO_Port, BP_LED_Pin, GPIO_PIN_SET);
        }

        osDelay(100);
    }
}

void dspTask( void * argument)
{
    /**********
     *
     * Number | PGFE.DCBA | Value
     *   0    | 0011.1111 | 3Fh
     *   1    | 0000.0110 | 06h
     *   2    | 0101.1011 | 5Bh
     *   3    | 0100.1111 | 4Fh
     *   4    | 0110.0110 | 66h
     *   5    | 0110.1101 | 6Dh
     *   6    | 0111.1101 | 7Dh
     *   7    | 0000.0111 | 07h
     *   8    | 0111.1111 | 7Fh
     *   9    | 0110.1111 | 6Fh
     *   A    | 0111.0111 | 77h
     *   B    | 0111.1100 | 7Ch
     *   C    | 0011.1001 | 39h
     *   D    | 0101.1110 | 5Eh
     *   E    | 0111.1001 | 79h
     *   F    | 0111.0001 | 71h
     *
     **********/
    //                          0      1      2      3      4      5      6      7      8      9      A      B      C      D      E      F
    uint8_t u8_numbers[] = { 0x3F , 0x06 , 0x5B , 0x4F , 0x66 , 0x6D , 0x7D , 0x07 , 0x7F , 0x6F , 0x77 , 0x7C , 0x39 , 0x5E , 0x79 , 0x71 } ;

    /**********
     *
     * Number | GPFE.DCBA | Value
     *   0    | 0011.1111 | 3Fh
     *   1    | 0000.0110 | 06h
     *   2    | 1001.1011 | 9Bh
     *   3    | 1000.1111 | 8Fh
     *   4    | 1010.0110 | A6h
     *   5    | 1010.1101 | ADh
     *   6    | 1011.1101 | BDh
     *   7    | 0000.0111 | 07h
     *   8    | 1011.1111 | BFh
     *   9    | 1010.1111 | AFh
     *   A    | 1011.0111 | B7h
     *   B    | 1011.1100 | BCh
     *   C    | 0011.1001 | 39h
     *   D    | 1001.1110 | 9Eh
     *   E    | 1011.1001 | B9h
     *   F    | 1011.0001 | B1h
     *
     **********/
    //                                0      1      2      3      4      5      6      7      8      9      A      B      C      D      E      F
    uint8_t u8_numUpSideDown[] = { 0x3F , 0x06 , 0x9B , 0x8F , 0xA6 , 0xAD , 0xBD , 0x07 , 0xBF , 0xAF , 0xB7 , 0xBC , 0x39 , 0x9E , 0xB9 , 0xB1 } ;


    uint8_t u8_count ;
    uint8_t u8_index ;
    uint8_t segment ;

    struct _DISPLAY_GPIO_
    {
        GPIO_TypeDef * GPIOx    ;
        uint16_t       GPIO_Pin ;
    } displayGpio[] = { { .GPIOx = BP_00_GPIO_Port , .GPIO_Pin = BP_00_Pin } ,
                        { .GPIOx = BP_01_GPIO_Port , .GPIO_Pin = BP_01_Pin } ,
                        { .GPIOx = BP_02_GPIO_Port , .GPIO_Pin = BP_02_Pin } ,
                        { .GPIOx = BP_03_GPIO_Port , .GPIO_Pin = BP_03_Pin } ,
                        { .GPIOx = BP_04_GPIO_Port , .GPIO_Pin = BP_04_Pin } ,
                        { .GPIOx = BP_05_GPIO_Port , .GPIO_Pin = BP_05_Pin } ,
                        { .GPIOx = BP_06_GPIO_Port , .GPIO_Pin = BP_06_Pin } ,
                        { .GPIOx = BP_07_GPIO_Port , .GPIO_Pin = BP_07_Pin } ,
                        { .GPIOx = BP_08_GPIO_Port , .GPIO_Pin = BP_08_Pin } ,
                        { .GPIOx = BP_09_GPIO_Port , .GPIO_Pin = BP_09_Pin } ,
                        { .GPIOx = BP_10_GPIO_Port , .GPIO_Pin = BP_10_Pin } ,
                        { .GPIOx = BP_11_GPIO_Port , .GPIO_Pin = BP_11_Pin } ,
                        { .GPIOx = BP_12_GPIO_Port , .GPIO_Pin = BP_12_Pin } ,
                        { .GPIOx = BP_13_GPIO_Port , .GPIO_Pin = BP_13_Pin } };

    struct _SEGMENT_GPIO_
    {
        GPIO_TypeDef * GPIOx    ;
        uint16_t       GPIO_Pin ;
    } segmentGpio[] = { { .GPIOx = BP_A_GPIO_Port , .GPIO_Pin = BP_A_Pin } ,
                        { .GPIOx = BP_B_GPIO_Port , .GPIO_Pin = BP_B_Pin } ,
                        { .GPIOx = BP_C_GPIO_Port , .GPIO_Pin = BP_C_Pin } ,
                        { .GPIOx = BP_D_GPIO_Port , .GPIO_Pin = BP_D_Pin } ,
                        { .GPIOx = BP_E_GPIO_Port , .GPIO_Pin = BP_E_Pin } ,
                        { .GPIOx = BP_F_GPIO_Port , .GPIO_Pin = BP_F_Pin } ,
                        { .GPIOx = BP_G_GPIO_Port , .GPIO_Pin = BP_G_Pin } ,
                        { .GPIOx = BP_P_GPIO_Port , .GPIO_Pin = BP_P_Pin } } ;

    for( u8_count = 0 ; u8_count < 14 ; u8_count++ )
    {
        HAL_GPIO_WritePin( displayGpio[ u8_count ].GPIOx , displayGpio[ u8_count ].GPIO_Pin, GPIO_PIN_RESET);
    }

    for( u8_count = 0 ; u8_count < 8 ; u8_count++ )
    {
        HAL_GPIO_WritePin( segmentGpio[ u8_count ].GPIOx , segmentGpio[ u8_count ].GPIO_Pin, GPIO_PIN_RESET);
    }

uint16_t timeX ;
uint8_t idx = 0 ;

    for( u8_index = 0 , timeX = 0 ; /* EVER */ ; )
    {
        timeX++ ;
        if( timeX >= 1000 )
        {
            timeX = 0 ;
            idx++ ;
        }

        osDelay(1);

        HAL_GPIO_WritePin( displayGpio[ u8_index ].GPIOx , displayGpio[ u8_index ].GPIO_Pin, GPIO_PIN_RESET);

        u8_index++ ;
        if( u8_index >= 14 )
        {
            u8_index = 0 ;
        }

        if( u8_index != 10 && u8_index != 12 )
        {
            segment = u8_numbers[ idx & 0x0F ] ;
        }
        else
        {
            segment = u8_numUpSideDown[ idx & 0x0F ] ;
        }

        for( u8_count = 0 ; u8_count < 8 ; u8_count++ )
        {
            GPIO_PinState pinState = ( segment & ( 0x01 << u8_count ) ) ? ( GPIO_PIN_SET ) : ( GPIO_PIN_RESET ) ;
            HAL_GPIO_WritePin( segmentGpio[ u8_count ].GPIOx , segmentGpio[ u8_count ].GPIO_Pin, pinState);
        }

        HAL_GPIO_WritePin( displayGpio[ u8_index ].GPIOx , displayGpio[ u8_index ].GPIO_Pin, GPIO_PIN_SET);
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
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

  uint8_t buf_tx[ 64 ] ;
  uint16_t buf_tx_size ;
  uint8_t u8_ret ;

  QueueHandle_t xQueue_time ;
  time_t epoch ;


  xQueue_time = ( QueueHandle_t ) argument ;

  /* Infinite loop */
  for( ; /* EVER */ ; )
  {
      if( xQueueReceive( xQueue_time , &epoch, 5000 ) == pdPASS)
      {
          buf_tx_size = sprintf( ( char * ) buf_tx , "%lu\n" , epoch ) ;
      }
      else
      {
          buf_tx_size = sprintf( ( char * ) buf_tx , "No time received!\n" ) ;
      }

      u8_ret = CDC_Transmit_FS( buf_tx , buf_tx_size ) ;
      if( USBD_OK != u8_ret )
      {
          // TODO!
      }
  }
  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
