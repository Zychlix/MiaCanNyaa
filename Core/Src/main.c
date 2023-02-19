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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <stdio.h>
#include <string.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_EGV_ACCEL_VAR_ID 0x201
#define CAN_EGV_CMD_VAR_ID 0x301
#define CAN_EGV_SYNC_ALL_ID 0x80

#define CAN_VAR_STAT
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */



uint16_t swap_endianness(uint16_t value)
{
    uint16_t ret_val;
    uint8_t lsb, msb;
    lsb = value & 0x00FF;
    msb = value & 0xFF00;
    ret_val = (lsb << 8) + (msb >> 8);

    return ret_val;

}


/* USER CODE END PM */

volatile uint8_t var_ready = 0;

typedef struct stat
{
    int32_t motor_speed;
    int16_t motor_torque;
    int16_t status_word;
}stat_t;
typedef struct CAN_EGV_Accel_VAR
{
    uint16_t accelerator_set_point;//little endian
    uint16_t regen_max;
    unsigned int forward : 1 ;
    unsigned int reverse : 1 ;
    unsigned int footbrake : 1;
    unsigned int DS1 : 1 ;
    unsigned int footswitch :1;
    unsigned int DS2 : 1;
    unsigned int unused :2;

} CAN_EGV_Accel_VAR_t;

typedef struct CAN_EGV_Cmd_VAR
{
    int16_t current_limit; //0 to 500
    int16_t regen_limit;
    uint16_t max_torque_ratio;
    uint16_t motor_command;

} CAN_EGV_Cmd_VAR_t;

typedef struct CAN_EGV_SYNC_ALL
{
uint8_t status;

} CAN_EGV_SYNC_ALL_t;

void can_send_egv_sync_all(CAN_EGV_SYNC_ALL_t * frame)
{
    CAN_TxHeaderTypeDef carrier = {0};
    carrier.StdId = CAN_EGV_SYNC_ALL_ID;
    carrier.DLC = sizeof (CAN_EGV_SYNC_ALL_t);
    HAL_CAN_AddTxMessage(&hcan1,&carrier,(char*)frame,NULL);

}

void can_send_egv_accel_var(CAN_EGV_Accel_VAR_t * frame)
{
    CAN_TxHeaderTypeDef carrier = {0};

    carrier.StdId = CAN_EGV_ACCEL_VAR_ID;
    carrier.DLC = sizeof (CAN_EGV_Accel_VAR_t);
    //frame->accelerator_set_point = swap_endianness(frame->accelerator_set_point);
    HAL_CAN_AddTxMessage(&hcan1,&carrier,(char *)frame,NULL);
}

void can_send_egv_cmd_var(CAN_EGV_Cmd_VAR_t * frame)
{
    CAN_TxHeaderTypeDef carrier = {0};
    carrier.StdId = CAN_EGV_CMD_VAR_ID;
    carrier.DLC = sizeof (CAN_EGV_Cmd_VAR_t);
    HAL_CAN_AddTxMessage(&hcan1,&carrier,(char *)frame,NULL);

}

uint16_t get_throttle()
{
    HAL_ADC_Start(&hadc1);
    uint32_t reading = HAL_ADC_GetValue(&hadc1);
    return (uint16_t)(reading>>4);
}

void update_accel_pedal(CAN_EGV_Accel_VAR_t * frame)
{
    int direction_forward = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)== GPIO_PIN_RESET);

    frame->accelerator_set_point = get_throttle();

    if(var_ready && frame->accelerator_set_point > 70) {
        frame->accelerator_set_point = 250;
        frame->footswitch = 1; //?
        frame->regen_max = 0;
        frame->footbrake =0;
        if(direction_forward)
        {
            frame->reverse = 0;
            frame->forward = 1;
        } else
        {
            frame->reverse=1;
            frame->forward = 0;
        }
    } else
    {
        frame->footswitch = 0; //?
//        frame->regen_max = 0;
        frame->forward = 0;
        frame->reverse = 0;
        frame->accelerator_set_point = 0;
    }
}
volatile CAN_EGV_Accel_VAR_t egv_accel_frame ={0};

volatile CAN_EGV_SYNC_ALL_t egv_sync_frame ={0};

volatile CAN_EGV_Cmd_VAR_t egv_var_frame ={0};

/* Private user code ---------------------------------------------------------*/
stat_t var_stat;

void diagnostics_print()
{
    printf("Var stat: %d\nvar status %d:", var_stat.motor_speed, var_stat.status_word);
    printf("Accel frame 0x201: \n");
    printf("Accel var values %d", egv_accel_frame);
    printf("accel setpoint: %d \nfootswitch: %d \nforward: %d \n\n" ,egv_accel_frame.accelerator_set_point, egv_accel_frame.footswitch, egv_accel_frame.forward);
}

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{

    update_accel_pedal(&egv_accel_frame);
    can_send_egv_sync_all(&egv_sync_frame);
    can_send_egv_accel_var(&egv_accel_frame);
    can_send_egv_cmd_var(&can_send_egv_cmd_var);

}

void HAL_CAN_RxFIFO0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
}

int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
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

  egv_accel_frame;

  egv_accel_frame.accelerator_set_point = 0;
  egv_accel_frame.footbrake=0;
  egv_accel_frame.footswitch = 0; //?
  egv_accel_frame.regen_max = 0;
  egv_accel_frame.forward = 0;

  egv_sync_frame.status =0xff;
  egv_var_frame.current_limit = 0;
  //egv_var_frame.current_limit = swap_endianness(egv_var_frame.current_limit);
  egv_var_frame.max_torque_ratio =0;
  //egv_var_frame.max_torque_ratio = swap_endianness(egv_var_frame.max_torque_ratio);
  egv_var_frame.motor_command = 0;
  egv_var_frame.regen_limit = 0;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */



    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
    sFilterConfig.FilterIdHigh=0x181<<5; //the ID that the filter looks for (switch this for the other microcontroller)
    sFilterConfig.FilterIdLow=0;
    sFilterConfig.FilterMaskIdHigh=0;
    sFilterConfig.FilterMaskIdLow=0;
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
    sFilterConfig.FilterActivation=ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig); //configure CAN filter

//
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    __HAL_RCC_ADC_CLK_ENABLE();
    HAL_ADC_Start(&hadc1);
//    HAL_CAN
    //HAL_CAN_Init()
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    diagnostics_print();
    /* USER CODE BEGIN 3 */
    if(var_stat.status_word == 1075)
    {
        var_ready =1;
        egv_var_frame.current_limit = 200; //2640
        egv_var_frame.regen_limit =20;
        //egv_var_frame.current_limit = swap_endianness(egv_var_frame.current_limit);
        egv_var_frame.max_torque_ratio =1000;
        egv_accel_frame.footswitch=1;
        egv_accel_frame.accelerator_set_point=250;
    }
     HAL_Delay(1000);
//
//     printf("%d \n",egv_accel_frame.accelerator_set_point);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 6400;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100; // every 10ms
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
