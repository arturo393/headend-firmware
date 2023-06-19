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
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

static const char *END_RX_DATA = "&";
static const char *START_RX_DATA = "$";
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define ADC_CONSUMPTION_CURRENT_FACTOR  0.06472492f
#define ADC_LINE_CURRENT_FACTOR  0.0010989f
#define ADC_VOLTAGE_FACTOR  0.01387755f
#define ARTERIAL1_VOLTAGE_OBJECT_NAME "x0\0"
#define ARTERIAL1_CURRENT_OBJECT_NAME "x1\0"
#define ARTERIAL1_POWER_OBJECT_NAME "x2\0"
#define ARTERIAL1_DC_ON_OBJECT_NAME "bt0\0"
#define ARTERIAL1_RF_ON_OBJECT_NAME "bt1\0"
#define ARTERIAL1_DC_ON_OBJECT_ID 10
#define ARTERIAL1_RF_ON_OBJECT_ID 5

#define ARTERIAL2_VOLTAGE_OBJECT_NAME "x5\0"
#define ARTERIAL2_CURRENT_OBJECT_NAME "x4\0"
#define ARTERIAL2_POWER_OBJECT_NAME "x3\0"
#define ARTERIAL2_DC_ON_OBJECT_NAME "bt2\0"
#define ARTERIAL2_RF_ON_OBJECT_NAME "bt3\0"
#define ARTERIAL2_DC_ON_OBJECT_ID 24
#define ARTERIAL2_RF_ON_OBJECT_ID 23

#define ARTERIAL3_VOLTAGE_OBJECT_NAME "x8\0"
#define ARTERIAL3_CURRENT_OBJECT_NAME "x7\0"
#define ARTERIAL3_POWER_OBJECT_NAME "x6\0"
#define ARTERIAL3_DC_ON_OBJECT_NAME "bt4\0"
#define ARTERIAL3_RF_ON_OBJECT_NAME "bt5\0"
#define ARTERIAL3_DC_ON_OBJECT_ID 26
#define ARTERIAL3_RF_ON_OBJECT_ID 25

#define ARTERIAL4_VOLTAGE_OBJECT_NAME "x11\0"
#define ARTERIAL4_CURRENT_OBJECT_NAME "x10\0"
#define ARTERIAL4_POWER_OBJECT_NAME "x9\0"
#define ARTERIAL4_DC_ON_OBJECT_NAME "bt6\0"
#define ARTERIAL4_RF_ON_OBJECT_NAME "bt7\0"
#define ARTERIAL4_DC_ON_OBJECT_ID 28
#define ARTERIAL4_RF_ON_OBJECT_ID 27

#define UPLINK_OBJECT_NAME "x13\0"
#define DOWNLINK_OBJECT_NAME "x12\0"

int DC_4, DC_3, DC_2, DC_1;
int ART_4;
int ART_3;
int ART_2;
int ART_1;
int alarm1;
int alarm2;
int alarm3;
int alarm4;
int alarm5;
int voltageSelected;
float currentArterial4;
float currentArterial3;
float currentArterial2;
float currentArterial1;
int uplinkLevel;
int downlinkLevel;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {
	ARTERIAL_1, ARTERIAL_2, ARTERIAL_3, ARTERIAL_4, ARTERIAL_NUMBER
} ARTERIAL_NUM_T;

typedef enum NEXTION_UART_INDEX {
	START, CMD, VALUE, END, BUTTON_DATA_SIZE
} NEXTION_INDEX_T;

typedef enum {
	ARTERIAL1_CURRENT,
	ARTERIAL4_CURRENT,
	VOLTAGE,
	DOWNLINK_LEVEL,
	UPLINK_LEVEL,
	ARTERIAL3_CURRENT,
	ARTERIAL2_CURRENT,
	ADC_CHANNELS
} ADC_CHANNEL_T;

typedef struct ARTERIAL {
	float voltage;
	float current;
	uint8_t power;
	char voltageObjectName[4];
	char currentObjectName[4];
	char powerObjectName[4];
	char rfOnObjectName[4];
	char dcOnObjectName[4];
	uint8_t voltageObjectID;
	uint8_t currentObjectID;
	uint8_t powerObjectID;
	uint8_t rfOnObjectID;
	uint8_t dcOnObjectID;
} Arterial;

typedef struct {
	GPIO_TypeDef *dcPort;
	uint16_t dcPin;
	GPIO_TypeDef *rfPort;
	uint16_t rfPin;
	bool dcOn;
	bool rfOn;
} ArterialIO;

Arterial arterial[ARTERIAL_NUMBER];
ArterialIO arterialIOs[ARTERIAL_NUMBER] = { { DC_ARTERIAL_1_GPIO_Port,
DC_ARTERIAL_1_Pin, RF_ARTERIAL_1_GPIO_Port, RF_ARTERIAL_1_Pin, false,
false }, { DC_ARTERIAL_2_GPIO_Port, DC_ARTERIAL_2_Pin,
RF_ARTERIAL_2_GPIO_Port, RF_ARTERIAL_2_Pin, false, false }, {
DC_ARTERIAL_3_GPIO_Port, DC_ARTERIAL_3_Pin, RF_ARTERIAL_3_GPIO_Port,
RF_ARTERIAL_3_Pin, false, false }, { DC_ARTERIAL_4_GPIO_Port,
DC_ARTERIAL_4_Pin, RF_ARTERIAL_4_GPIO_Port, RF_ARTERIAL_4_Pin, false,
false } };

uint8_t uart1RxData[BUTTON_DATA_SIZE];
uint8_t Cmd_End[3] = { 0xff, 0xff, 0xff };
bool sendData = false;

uint16_t adcValues[ADC_CHANNELS];

void sendNumToNextion(char *obj, int32_t num) {
	uint8_t buffer[30] = { 0 };
	int len = sprintf((char*) buffer, "%s.val=%ld", obj, num);
	HAL_UART_Transmit(&huart1, buffer, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
}

void sendFloatToNextion(char *obj, float num, int dp) {
	// convert to the integer
	uint8_t len = 0;
	int32_t number = num * (pow(10, dp));
	HAL_StatusTypeDef res;
	uint8_t buffer[30] = { 0 };
	len = sprintf((char*) buffer, "%s.vvs1=%d", obj, dp);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
	len = sprintf((char*) buffer, "%s.val=%ld", obj, number);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);

}

void sendArterialToNextion(Arterial arterial) {
	sendFloatToNextion(arterial.voltageObjectName, arterial.voltage, 2);
	sendFloatToNextion(arterial.currentObjectName, arterial.current, 2);
	sendFloatToNextion(arterial.powerObjectName, arterial.power, 2);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if ((uart1RxData[START] == *START_RX_DATA)
			&& (uart1RxData[END] == *END_RX_DATA)) {
		for (uint8_t i = 0; i < ARTERIAL_NUMBER; i++) {
			if (uart1RxData[CMD] == arterial[i].dcOnObjectID)
				arterialIOs[i].dcOn = uart1RxData[VALUE];

			else if (uart1RxData[CMD] == arterial[i].rfOnObjectID)
				arterialIOs[i].rfOn = uart1RxData[VALUE];
		}
		sendData = true;
	}
	huart->RxXferCount = 0;
	HAL_UART_Receive_IT(huart, uart1RxData, BUTTON_DATA_SIZE);
}

void NEXTION_SendQR(char *obj, char *str) {
	uint8_t *buffer = malloc(80 * sizeof(char));

	HAL_UART_Transmit(&huart1, (uint8_t*) "page 1", 6, 100);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);

	int len = sprintf((char*) buffer, "%s.txt=\"%s\"", obj, str);
	HAL_UART_Transmit(&huart1, buffer, len, 1000);
	HAL_UART_Transmit(&huart1, Cmd_End, 3, 100);
	free(buffer);
}

uint8_t RxData = 0;
int32_t num = 1;
float value = 1.1;

void arterialInit(Arterial arterial[ARTERIAL_NUMBER],
		ArterialIO arterialIOs[ARTERIAL_NUMBER]) {
	const char *voltageObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_VOLTAGE_OBJECT_NAME, ARTERIAL2_VOLTAGE_OBJECT_NAME,
	ARTERIAL3_VOLTAGE_OBJECT_NAME, ARTERIAL4_VOLTAGE_OBJECT_NAME };
	const char *currentObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_CURRENT_OBJECT_NAME, ARTERIAL2_CURRENT_OBJECT_NAME,
	ARTERIAL3_CURRENT_OBJECT_NAME, ARTERIAL4_CURRENT_OBJECT_NAME };
	const char *powerObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_POWER_OBJECT_NAME, ARTERIAL2_POWER_OBJECT_NAME,
	ARTERIAL3_POWER_OBJECT_NAME, ARTERIAL4_POWER_OBJECT_NAME };
	const char *dcOnObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_DC_ON_OBJECT_NAME, ARTERIAL2_DC_ON_OBJECT_NAME,
	ARTERIAL3_DC_ON_OBJECT_NAME, ARTERIAL4_DC_ON_OBJECT_NAME };
	const char *rfOnObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_RF_ON_OBJECT_NAME, ARTERIAL2_RF_ON_OBJECT_NAME,
	ARTERIAL3_RF_ON_OBJECT_NAME, ARTERIAL4_RF_ON_OBJECT_NAME };
	const int dcObjectIDs[ARTERIAL_NUMBER] = { ARTERIAL1_DC_ON_OBJECT_ID,
	ARTERIAL2_DC_ON_OBJECT_ID, ARTERIAL3_DC_ON_OBJECT_ID,
	ARTERIAL4_DC_ON_OBJECT_ID };
	const int rfObjectIDs[ARTERIAL_NUMBER] = { ARTERIAL1_RF_ON_OBJECT_ID,
	ARTERIAL2_RF_ON_OBJECT_ID, ARTERIAL3_RF_ON_OBJECT_ID,
	ARTERIAL4_RF_ON_OBJECT_ID };
	for (uint8_t i = 0; i < ARTERIAL_NUMBER; i++) {
		sprintf(arterial[i].voltageObjectName, voltageObjectNames[i]);
		sprintf(arterial[i].currentObjectName, currentObjectNames[i]);
		sprintf(arterial[i].powerObjectName, powerObjectNames[i]);
		sprintf(arterial[i].dcOnObjectName, dcOnObjectNames[i]);
		sprintf(arterial[i].rfOnObjectName, rfOnObjectNames[i]);
		arterial[i].dcOnObjectID = dcObjectIDs[i];
		arterial[i].rfOnObjectID = rfObjectIDs[i];
		arterialIOs[i].dcOn = false;
		arterialIOs[i].rfOn = false;
	}
}

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {
		TIM2->SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag
		sendData = true;

	}
}

void TIM2_Interrupt_Init(void) {
	NVIC_SetPriority(TIM2_IRQn, 0);  // Set the interrupt priority
	NVIC_EnableIRQ(TIM2_IRQn);  // Enable the interrupt
}

void startTimer2(void) {
	TIM2->CR1 |= TIM_CR1_CEN;  // Start the timer
}

void enableGlobalInterrupts(void) {
	__enable_irq();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	HAL_StatusTypeDef res;
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
	MX_ADC_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	// Enable the timer clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// Configure the timer
	TIM2->PSC = 7999; // Prescaler value for a 1 ms time base (assuming a 48 MHz clock)
	TIM2->ARR = 499;  // Auto-reload value for a 500 ms period
	TIM2->ARR = 3999;  // Auto-reload value for a 500 ms period
	TIM2->CR1 |= TIM_CR1_ARPE;  // Enable auto-reload preload
	TIM2->CR1 |= TIM_CR1_URS; // Only overflow/underflow generates an update interrupt
	TIM2->DIER |= TIM_DIER_UIE;  // Enable update interrupt

	// Initialize necessary peripherals and configurations
	TIM2_Interrupt_Init();
	enableGlobalInterrupts();

	res = HAL_ADC_Start_DMA(&hadc, (uint32_t*) adcValues, ADC_CHANNELS);
	if (res != HAL_OK)
		Error_Handler();

	res = HAL_UART_Receive_IT(&huart1, uart1RxData, BUTTON_DATA_SIZE);
	if (res != HAL_OK)
		Error_Handler();

	uint32_t nextionSendTicks = HAL_GetTick();
	uint32_t nextionSendTimeout = 1000;

	arterialInit(arterial, arterialIOs);
	startTimer2();

	srand(10000);
	// ...
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		for (int i = 0; i < ARTERIAL_NUMBER; i++) {

			HAL_GPIO_WritePin(arterialIOs[i].dcPort, arterialIOs[i].dcPin,
					arterialIOs[i].dcOn);
			HAL_GPIO_WritePin(arterialIOs[i].rfPort, arterialIOs[i].rfPin,
					arterialIOs[i].rfOn);

			if (arterialIOs[i].dcOn) {
				arterial[i].voltage = adcValues[VOLTAGE] * ADC_VOLTAGE_FACTOR;
				arterial[ARTERIAL_4].current = adcValues[ARTERIAL4_CURRENT]
						* ADC_CONSUMPTION_CURRENT_FACTOR;	//VARIABLE ANALOGA
				arterial[ARTERIAL_3].current = adcValues[BUTTON_DATA_SIZE]
						* ADC_CONSUMPTION_CURRENT_FACTOR;	//VARIABLE ANALOGA
				arterial[ARTERIAL_2].current = adcValues[ARTERIAL2_CURRENT]
						* ADC_CONSUMPTION_CURRENT_FACTOR;	//VARIABLE ANALOGA
				arterial[ARTERIAL_1].current = adcValues[ARTERIAL1_CURRENT]
						* ADC_CONSUMPTION_CURRENT_FACTOR;	//VARIABLE ANALOGA
				downlinkLevel = adcValues[DOWNLINK_LEVEL];	//VARIABLE ANALOGA
				uplinkLevel = adcValues[UPLINK_LEVEL];		//VARIABLE ANALOGA
				arterial[i].power = arterial[i].voltage * arterial[i].current;
			} else {
				arterial[i].voltage = (rand() % 48);
				arterial[i].current = (rand() % 5);
				arterial[i].power = arterial[i].voltage * arterial[i].current;
				downlinkLevel = rand() % 45;	//VARIABLE ANALOGA
				uplinkLevel = rand() % 45;	//VARIABLE ANALOGA
			}
		}

		if (sendData) {
			sendData = false;
			for (int i = 0; i < ARTERIAL_NUMBER; i++)
				sendArterialToNextion(arterial[i]);
			sendFloatToNextion(DOWNLINK_OBJECT_NAME, downlinkLevel, 2);
			sendFloatToNextion(UPLINK_OBJECT_NAME, uplinkLevel, 2);
		}

		alarm1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);	//ESTADO ALARM1
		alarm2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);	//ESTADO ALARM2
		alarm3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);	//ESTADO ALARM3
		alarm4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);	//ESTADO ALARM4
		alarm5 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);	//ESTADO ALARM5

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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
	sConfig.Channel = ADC_CHANNEL_10;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x2000090E;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			DC_ARTERIAL_1_Pin | DC_ARTERIAL_2_Pin | DC_ARTERIAL_3_Pin
					| DC_ARTERIAL_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			RF_ARTERIAL_4_Pin | RF_ARTERIAL_3_Pin | RF_ARTERIAL_2_Pin
					| GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
					| RF_ARTERIAL_1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : DC_ARTERIAL_1_Pin DC_ARTERIAL_2_Pin DC_ARTERIAL_3_Pin DC_ARTERIAL_4_Pin */
	GPIO_InitStruct.Pin = DC_ARTERIAL_1_Pin | DC_ARTERIAL_2_Pin
			| DC_ARTERIAL_3_Pin | DC_ARTERIAL_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RF_ARTERIAL_4_Pin RF_ARTERIAL_3_Pin RF_ARTERIAL_2_Pin PB12
	 PB13 PB14 PB15 RF_ARTERIAL_1_Pin */
	GPIO_InitStruct.Pin = RF_ARTERIAL_4_Pin | RF_ARTERIAL_3_Pin
			| RF_ARTERIAL_2_Pin | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
			| GPIO_PIN_15 | RF_ARTERIAL_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB11 PB4 PB5
	 PB6 PB7 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_4 | GPIO_PIN_5
			| GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PC8 PC9 PC10 PC11
	 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
			| GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
