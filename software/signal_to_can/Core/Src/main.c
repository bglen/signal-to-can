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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "can_module.h"
#include "adc_module.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	volatile HAL_StatusTypeDef last_tx_status; /* return from CAN_Module_Send */
	volatile uint32_t last_hal_error; /* HAL_CAN_GetError() */
	volatile uint32_t last_esr; /* CAN->ESR snapshot */
	volatile uint32_t last_tsr; /* CAN->TSR snapshot */
	volatile uint32_t tx_ok_count; /* TX mailbox complete count */
	volatile uint32_t tx_error_count; /* HAL error callback count */
} can_debug_t;

extern can_debug_t g_can_dbg;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Heart beat
#define HEARTBEAT_INTERVAL_MS 500

// CAN bus

// ADC
#define ADC_SAMPLE_RATE_HZ  100u   /* per-channel Hz; change as needed */
#define ADC_VREF_MV         3300u  /* board Vref in millivolts */
#define ADC_DIVIDER_SCALE   3.63636f

// Test
#define TEST_CAN_ID 0x124
#define TEST_SEND_INTERVAL_MS    1000u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

// Heart beat
static uint32_t heartbeat_tick = 0;

// CAN bus
uint32_t baud_enum = 2; // 2 -> 500 kbps

// ADC

// Test
can_debug_t g_can_dbg = { 0 };
static uint32_t last_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

void Heartbeat_Task(void);
static void Test_Can_Task(uint16_t value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_CAN_Init();
	MX_ADC_Init();
	/* USER CODE BEGIN 2 */

	// Set Standby Pin on CAN transceiver low (normal mode) before initializing
	HAL_GPIO_WritePin(GPIOB, CAN_STANDBY_Pin, GPIO_PIN_RESET);

	if (CAN_Module_Init(&hcan, baud_enum) != HAL_OK) {
		// TODO
		Error_Handler();
	}

	if (ADC_Module_Init(&hadc, ADC_VREF_MV, ADC_SAMPLE_RATE_HZ) != HAL_OK) {
		// TODO
		Error_Handler();
	}

	// Set ADC to default settings
	ADC_Module_Set_Enable_Mask(0xFFu);
	for (uint8_t ch = 0; ch < 8u; ch++) {
		ADC_Module_Set_Scale(ch, 1.0f);
	}

	last_tick = HAL_GetTick();
	heartbeat_tick = last_tick;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		Heartbeat_Task();
		ADC_Module_Task();
		float ch0_v = ADC_Module_Get_Pin_Voltage(0);
		uint16_t ch0_v_i = (uint16_t)(ch0_v * 1000); // get channel 0 voltage in whole mV
		Test_Can_Task(ch0_v_i);
		HAL_Delay(1);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 6;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			LED_STATUS_1_Pin | LED_STATUS_2_Pin | CAN_STANDBY_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_STATUS_1_Pin LED_STATUS_2_Pin CAN_STANDBY_Pin */
	GPIO_InitStruct.Pin = LED_STATUS_1_Pin | LED_STATUS_2_Pin | CAN_STANDBY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Heartbeat_Task(void) {
	/* Check if enough time has passed */
	if ((HAL_GetTick() - heartbeat_tick) >= HEARTBEAT_INTERVAL_MS) {
		/* Toggle the LED */
		HAL_GPIO_TogglePin(GPIOB, LED_STATUS_1_Pin); // Example: Port B

		/* Update last toggle time */
		heartbeat_tick = HAL_GetTick();
	}
}

/**
 * @brief Periodically send a simple CAN test message and flash LED on success.
 *        Note: Updated to use CAN_Module_Send_Std() from can_module.c.
 */
static void Test_Can_Task(uint16_t value) {
	uint32_t now = HAL_GetTick();
	if ((now - last_tick) < TEST_SEND_INTERVAL_MS) {
		return;
	}
	last_tick = now;

	/* Minimal 1-byte test payload (kept as-is). */
	uint8_t data[2] = {(uint8_t)((value >> 8) & 0xFFu), (uint8_t)(value & 0xFFu)};


	/* Short TX timeout so we don't block the task loop. */
	const uint32_t tx_timeout_ms = 10u;

	/* Updated call: (std_id, data_ptr, dlc, timeout_ms). */
	HAL_StatusTypeDef st = CAN_Module_Send_Std(TEST_CAN_ID, data, 2u, tx_timeout_ms);

	g_can_dbg.last_tx_status = st;
	g_can_dbg.last_hal_error = HAL_CAN_GetError(&hcan);
	g_can_dbg.last_esr = hcan.Instance->ESR;
	g_can_dbg.last_tsr = hcan.Instance->TSR;

	if (st == HAL_OK) {
		HAL_GPIO_TogglePin(GPIOB, LED_STATUS_2_Pin);
	}
}

/* Called when a mailbox actually finishes transmitting */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *h) {
	(void) h;
	g_can_dbg.tx_ok_count++;
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *h) {
	(void) h;
	g_can_dbg.tx_ok_count++;
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *h) {
	(void) h;
	g_can_dbg.tx_ok_count++;
}

/* Called on any CAN error (ACK error, bus-off, etc.) */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *h) {
	g_can_dbg.last_hal_error = HAL_CAN_GetError(h);
	g_can_dbg.last_esr = h->Instance->ESR; /* includes TEC/REC/LEC */
	g_can_dbg.last_tsr = h->Instance->TSR;
	g_can_dbg.tx_error_count++;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
