/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum mc_wipers_state
{
	WIPERS_PARKED = 0,
	WIPERS_ON,
	WIPERS_REQUEST_STOP
};

enum mc_wipers_speed
{
	WIPERS_OFF,
	WIPERS_SLOW,
	WIPERS_MEDIUM,        /* TODO: Not implemented yet */
	WIPERS_FAST
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PARK_AFTER_CHECK_TIME ((int)80) /* 8ms */
#define CAN_RX_STD_ID         0x123U
#define SW_VERSION            "0.1"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
atomic_bool g_park_rising_edge_fired;
atomic_bool g_timer_elapsed;
atomic_int  g_wipers_speed;
atomic_bool g_timer_slow_period_elapsed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* User Interrupts */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	atomic_store(&g_park_rising_edge_fired, true);
	//HAL_UART_Transmit(&huart3, "edge\r\n", 6, 10);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7) {
		/* Dummy end */
		//HAL_UART_Transmit(&huart3, "timer\r\n", 7, 10);
		atomic_store(&g_timer_elapsed, true);
	} else if (htim->Instance == TIM6) {
		HAL_UART_Transmit(&huart3, "timer6\r\n", 8, 10);
		atomic_store(&g_timer_slow_period_elapsed, true);
		HAL_TIM_Base_Stop_IT(&htim6);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1)
    	return;

    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
        return;

    /* Byte 0 = speed (0-3), clamp to valid range */
    uint8_t raw_speed = RxData[0];
    if (raw_speed > 3) {
    	raw_speed = 3;
    }

    atomic_store(&g_wipers_speed, raw_speed);
    /*char dbg[30];
    int len = sprintf(dbg, "CAN %d\r\n", raw_speed);
    HAL_UART_Transmit(&huart3, dbg, len, 10);*/
}

void wipers_set_speed(enum mc_wipers_speed speed)
{
	switch (speed) {
	case WIPERS_SLOW:
	case WIPERS_MEDIUM:
		HAL_GPIO_WritePin(FAST_INH_PORT, FAST_INH_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SLOW_INH_PORT, SLOW_INH_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SLOW_PWM_PORT, SLOW_PWM_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(FAST_PWM_PORT, FAST_PWM_PIN, GPIO_PIN_RESET);
		break;
	case WIPERS_FAST:
		HAL_GPIO_WritePin(FAST_INH_PORT, FAST_INH_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SLOW_INH_PORT, SLOW_INH_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SLOW_PWM_PORT, SLOW_PWM_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(FAST_PWM_PORT, FAST_PWM_PIN, GPIO_PIN_SET);
		break;
	default: /* Ignore OFF */
		break;
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
  bool check_park_after = false;
  atomic_store(&g_park_rising_edge_fired, false);
  atomic_store(&g_timer_elapsed, false);
  atomic_store(&g_timer_slow_period_elapsed, false);
  enum mc_wipers_state wipers_state = WIPERS_PARKED;
  atomic_store(&g_wipers_speed, WIPERS_OFF);
  const char *fw_hello = "[MockCar] FW Version " SW_VERSION "\r\n";
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;      /* exact ID match */
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh         = CAN_RX_STD_ID << 5;        /* StdId in [15:5] */
  filter.FilterIdLow          = 0x0000;
  filter.FilterMaskIdHigh     = 0x0000;
  filter.FilterMaskIdLow      = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation     = ENABLE;
  filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &filter);

  HAL_CAN_Start(&hcan1);
  /* Enable FIFO0 message-pending interrupt */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart3, (const uint8_t*) fw_hello, strlen(fw_hello), 10);

  int loop_cnt = 0;
  bool slow_wait_active = false;
  bool _true = true;
  bool _false = false;
  while (1)
  {
	if (loop_cnt++ == 100000) {
		HAL_UART_Transmit(&huart3, (const uint8_t*) "LOOP ALIVE\r\n", 12, 10);
		loop_cnt = 0;
	}
	bool stop_motor = false;
	_true = true;
	_false = false;

	enum mc_wipers_speed wipers_speed = atomic_load(&g_wipers_speed);
	/* Check the park edge and start a delay timer */
	if (atomic_load(&g_park_rising_edge_fired) && !check_park_after) {
		/* Don't clear the park flag, that should be cleared after a small delay after debouncing (using timers). */
		HAL_TIM_Base_Stop(&htim7);
		check_park_after = true;
		TIM7->CNT = 0;
		HAL_TIM_Base_Start_IT(&htim7);
		atomic_store(&g_park_rising_edge_fired, false);
	}
	if (check_park_after) {
		if (atomic_load(&g_timer_elapsed)) {
			atomic_store(&g_timer_elapsed, false);
			HAL_TIM_Base_Stop(&htim7);
			check_park_after = false;
			/* Check the polarity of the edge */
			if (HAL_GPIO_ReadPin(PARK_SENSE_PORT, PARK_SENSE_PIN) == GPIO_PIN_SET) {
				/* This means the motor is parked in the park position. However,
				 * stop the motor if it's requested, otherwise not.
				 */
				if (wipers_speed == WIPERS_OFF || wipers_speed == WIPERS_SLOW) {
					stop_motor = true;
				}
			}
		}
	}


	if (atomic_compare_exchange_strong(&g_timer_slow_period_elapsed, &_true, false)) {
		HAL_UART_Transmit(&huart3, "SET\r\n", 5, 10);
		slow_wait_active = false;
	}

	switch (wipers_state) {
	case WIPERS_PARKED:
		/* If the wipers are parked and there's request to turn on the wipers, start it.
		 * However, if are waiting for the SLOW period timer to finish, stay here */
		if (wipers_speed == WIPERS_OFF) {
			/* In case we go from the OFF state to SLOW, we need to disable the slow wait active */
			slow_wait_active = false;
		} else if (wipers_speed > WIPERS_SLOW) {
			wipers_state = WIPERS_ON;
			wipers_set_speed(wipers_speed);
		} else if (wipers_speed == WIPERS_SLOW && !slow_wait_active) {
			HAL_UART_Transmit(&huart3,  "Going to STOP\r\n", 16, 10);
			wipers_state = WIPERS_REQUEST_STOP;
			wipers_set_speed(WIPERS_SLOW);
			slow_wait_active = true;
		}
		break;
	case WIPERS_ON:
		wipers_set_speed(wipers_speed);
		if (wipers_speed == WIPERS_OFF || wipers_speed == WIPERS_SLOW) {
			wipers_state = WIPERS_REQUEST_STOP;
		}
		break;
	case WIPERS_REQUEST_STOP:
		/* If by any change a request to launch is received again, go back to the WIPERS_ON state */
		if (wipers_speed > WIPERS_SLOW) {
			wipers_state = WIPERS_ON;
		} else {
			if (stop_motor) {
				/* Caught stop motor condition, disable everything */
				HAL_GPIO_WritePin(SLOW_PWM_PORT, SLOW_PWM_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(FAST_PWM_PORT, FAST_PWM_PIN, GPIO_PIN_RESET);
				wipers_state = WIPERS_PARKED;
				if (wipers_speed == WIPERS_SLOW) {
					/* Start the countdown slow timer */
					HAL_TIM_Base_Stop(&htim6);
					TIM6->CNT = 0;
					HAL_TIM_Base_Start_IT(&htim6);
					slow_wait_active = true;
				}
			}
		}
		break;
	}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  htim6.Init.Prescaler = 1599;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1599;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 80;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
