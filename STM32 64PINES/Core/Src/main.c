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
#include <limits.h>
#include "rdss.h"
#include "module.h"

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

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_CONSUMPTION_CURRENT_FACTOR  0.06472492f
#define ADC_LINE_CURRENT_FACTOR  0.0010989f
#define ADC_VOLTAGE_FACTOR  0.01387755f
#define ARTERIAL1_VOLTAGE_OBJECT_NAME "x0\0"

#define ARTERIAL1_CURRENT_OBJECT_NAME "x1\0"
#define ARTERIAL1_CURRENT_BAR_NAME "j0\0"
#define ARTERIAL1_POWER_OBJECT_NAME "x2\0"
#define ARTERIAL1_DC_ON_OBJECT_NAME "bt1\0"
#define ARTERIAL1_RF_ON_OBJECT_NAME "bt0\0"
#define ARTERIAL1_DC_ON_OBJECT_ID 5
#define ARTERIAL1_RF_ON_OBJECT_ID 3
#define ARTERIAL1_CURRENT_OBJECT_ID 4

#define ARTERIAL2_VOLTAGE_OBJECT_NAME "x5\0"
#define ARTERIAL2_CURRENT_OBJECT_NAME "x4\0"
#define ARTERIAL2_CURRENT_BAR_NAME "j1\0"
#define ARTERIAL2_POWER_OBJECT_NAME "x3\0"
#define ARTERIAL2_DC_ON_OBJECT_NAME "bt3\0"
#define ARTERIAL2_RF_ON_OBJECT_NAME "bt2\0"
#define ARTERIAL2_DC_ON_OBJECT_ID 13
#define ARTERIAL2_RF_ON_OBJECT_ID 12
#define ARTERIAL2_CURRENT_OBJECT_ID 6

#define ARTERIAL3_VOLTAGE_OBJECT_NAME "x8\0"
#define ARTERIAL3_CURRENT_OBJECT_NAME "x7\0"
#define ARTERIAL3_CURRENT_BAR_NAME "j2\0"
#define ARTERIAL3_POWER_OBJECT_NAME "x6\0"
#define ARTERIAL3_DC_ON_OBJECT_NAME "bt5\0"
#define ARTERIAL3_RF_ON_OBJECT_NAME "bt4\0"
#define ARTERIAL3_DC_ON_OBJECT_ID 15
#define ARTERIAL3_RF_ON_OBJECT_ID 14
#define ARTERIAL3_CURRENT_OBJECT_ID 7

#define ARTERIAL4_VOLTAGE_OBJECT_NAME "x11\0"
#define ARTERIAL4_CURRENT_OBJECT_NAME "x10\0"
#define ARTERIAL4_CURRENT_BAR_NAME "j3\0"
#define ARTERIAL4_POWER_OBJECT_NAME "x9\0"
#define ARTERIAL4_DC_ON_OBJECT_NAME "bt7\0"
#define ARTERIAL4_RF_ON_OBJECT_NAME "bt6\0"
#define ARTERIAL4_DC_ON_OBJECT_ID 17
#define ARTERIAL4_RF_ON_OBJECT_ID 16
#define ARTERIAL4_CURRENT_OBJECT_ID 8

#define DC_48V_OBJECT_NAME "t4\0"
#define DC_24V_OBJECT_NAME "t5\0"
#define DC_12V_OBJECT_NAME "t6\0"
#define DC_BAR_OBJ "j5\0"
#define UPLINK_OBJECT_NAME "x13\0"
#define DOWNLINK_OBJECT_NAME "x12\0"
#define DL_BAR_OBJ "j4\0"

#define RDSS_FRAME_SIZE 14
#define SIGMA_FRAME_SIZE 14
#define RDSS_START_MARK 0x7e
#define RDSS_END_MARK  0x7f
#define RDSS_BUFFER_SIZE 50
#define LTEL_SET_LENGTH  13
#define LTEL_QUERY_LENGTH  9
#define MINIMUN_FRAME_LEN 6
#define CRC_HIGH_BYTE_OFFSET 2
#define CRC_LOW_BYTE_OFFSET 3
#define FRAME_HEADER_SIZE 4
#define QUERY_SIZE_DATA
#define UART2_RX_BUFFLEN 10
#define UART2_TX_BUFFLEN 30

#define ADC_MIN_VALUE 865
#define ADC_MAX_VALUE 1795
#define VOLTAGE_MIN 11.68
#define VOLTAGE_MAX 23.27

#define ADC_CURRENT_MIN_VALUE 165
#define ADC_CURRENT_MAX_VALUE 564
#define CURRENT_MIN_VALUE 0.34
#define CURRENT_MAX_VALUE 1.16

#define ADC_CURRENT_MIN_VALUE_old 418
#define ADC_CURRENT_MAX_VALUE_old 2866
#define CURRENT_MIN_VALUE_old 0.2
#define CURRENT_MAX_VALUE_old 1.22

// Downlink ADC Values

#define ADC_DOWNLINK_MAX 917
#define DOWNLINK_LEVEL_MAX -11
#define ADC_DOWNLINK_MIN 489
#define DOWNLINK_LEVEL_MIN -25.7
#define DOWNLINK_THRESHOLD 1230

#define UPLINK_LEVEL_MIN -45
#define UPLINK_LEVEL_MAX 0

#define ART1_THRESHOLD 450
#define ART2_THRESHOLD 450
#define ART3_THRESHOLD 450
#define ART4_THRESHOLD 450
#define VOLT_THRESHOLD 1232

#define UPLINK_THRESHOLD 100

#define VOLTAGE_THRESHOLD_5V 500
#define VOLTAGE_THRESHOLD_18V 1330
#define VOLTAGE_THRESHOLD_36V 2740

#define RED_C 63488
#define WHITE_C 65535
#define GREEN_BACKGROUND 1024
#define GREEN_DARK_BACKGROUND 480
#define GREY_BACKGROUND 50712
#define ORANGE_C 64128
#define GREY_C 21162
#define BLACK_C 0

#define ADC_WINDOW_SIZE 50

typedef enum {
	VIN_12V, VIN_24V, VIN_48V, VIN_UNKNOW
} VOLTAGE_STATE_T;

int uplinkLevel;
float downlinkLevel;
uint8_t dcBar;
uint8_t dlBar;
char dlBarObj[4];
bool vAlarm = false;
bool dlAlarm = false;
bool ulAlarm = false;
GPIO_PinState pinDcAlarm;
GPIO_PinState pinDownlinkAlarm;
char dcValueObjName[4];
char dc12vObjt[4];
char dc24vObjt[4];
char dc48vObjt[4];
char dcBarObjt[4];
VOLTAGE_STATE_T dcState; // 0  = 12[v] , 1 = 24[
float dcValue;

typedef enum {
	ART1, ART2, ART3, ART4, ARTERIAL_NUMBER
} ARTERIAL_NUM_T;

typedef enum NEXTION_UART_INDEX {
	START, CMD, VALUE, END, BUTTON_DATA_SIZE
} NEXTION_INDEX_T;

typedef enum {
	ART1_CURRENT_CH,
	ART4_CURRENT_CH,
	VOLTAGE_CH,
	DOWNLINK_LVL_CH,
	UPLINK_LVL_CH,
	ART3_CURRENT_CH,
	ART2_CURRENT_CH,
	ADC_CHANNELS
} ADC_CHANNEL_T;

typedef struct ARTERIAL {

	float current;
	uint8_t cBar;
	char cObj[4];
	char cBarObj[4];
	char rfObjName[4];
	char dcObjName[4];
	uint8_t vObjID;
	uint8_t cObjID;
	uint8_t pObjID;
	uint8_t rfObjID;
	uint8_t dcObjID;
	bool cAlarm;
	uint32_t dcAlarmColor;
	uint32_t cAlarmColor;
} Arterial;

// Create a lookup table to map arterial numbers to ADC indices
const int arterialToADCIndex[ARTERIAL_NUMBER] = { ART1_CURRENT_CH,
		ART2_CURRENT_CH, ART3_CURRENT_CH, ART4_CURRENT_CH };

typedef struct {
	GPIO_TypeDef *dcPort;
	uint16_t dcPin;
	GPIO_TypeDef *rfPort;
	uint16_t rfPin;
	GPIO_TypeDef *AlarmPort;
	uint16_t AlarmPin;
	bool dcOn;
	bool dcOnLast;
	bool rfOn;
	bool alarm;
} ArterialIO;

Arterial arterial[ARTERIAL_NUMBER];
ArterialIO arterialIOs[ARTERIAL_NUMBER];

// UART1 variables for nextion comunication
uint8_t uart1RxData[20];
uint8_t uart1RxSize;
uint8_t Cmd_End[3] = { 0xff, 0xff, 0xff };
bool sendDataOnTimeout = false;
bool isCmdOk = false;
bool isRxDataReady = false;

// UART2 variables for parameter quering
uint8_t rxData;
uint8_t uart2RxSize = 0;
uint8_t uart2RxData[UART2_RX_BUFFLEN];
bool isRxDataOk = false;
uint8_t txData;
uint8_t uart2TxIndex = 0;
uint8_t uart2TxData[UART2_TX_BUFFLEN];

uint32_t adcValues[ADC_CHANNELS];
uint8_t adcCounter[ADC_CHANNELS] = { 0 };
uint16_t adcReadings[ADC_CHANNELS][ADC_WINDOW_SIZE] = { 0 };
uint16_t adcMA[ADC_CHANNELS];
uint32_t adcSum[ADC_CHANNELS] = { 0 };

void intToNextion(char *obj, int32_t num) {
	uint8_t buffer[30] = { 0 };
	int len = sprintf((char*) buffer, "%s.val=%ld", obj, num);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
}

void floatToNextion(char *obj, float num, int dp) {
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

void colorToNextion(char *obj, uint32_t color) {
	// convert to the integer
	uint8_t len = 0;
	uint8_t buffer[30] = { 0 };
	len = sprintf((char*) buffer, "%s.bco=%lu", obj, color);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
}

void colorToNextionOn(char *obj, uint32_t color) {
	// convert to the integer
	uint8_t len = 0;
	uint8_t buffer[30] = { 0 };
	len = sprintf((char*) buffer, "%s.bco2=%lu", obj, color);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
}

void colorToNextionOff(char *obj, uint32_t color) {
	// convert to the integer
	uint8_t len = 0;
	uint8_t buffer[30] = { 0 };
	len = sprintf((char*) buffer, "%s.bco=%lu", obj, color);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
}

void textColorToNextion(char *obj, uint32_t color) {
	// convert to the integer
	uint8_t len = 0;
	uint8_t buffer[30] = { 0 };
	len = sprintf((char*) buffer, "%s.pco=%lu", obj, color);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
}

void sendArterialToNextion(Arterial arterial) {
	floatToNextion(arterial.cObj, arterial.current, 2);
	if (arterial.cAlarm)
		colorToNextion(arterial.cObj, RED_C);
	else
		colorToNextion(arterial.cObj, WHITE_C);

}

HAL_StatusTypeDef rxHalRes;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart == &huart1) {
		if ((uart1RxData[START] == *START_RX_DATA)
				&& (uart1RxData[END] == *END_RX_DATA)) {

			for (uint8_t i = 0; i < ARTERIAL_NUMBER; i++) {
				if (uart1RxData[CMD] == arterial[i].dcObjID) {
					arterialIOs[i].dcOn = uart1RxData[VALUE];
					HAL_GPIO_WritePin(arterialIOs[i].dcPort,
							arterialIOs[i].dcPin, arterialIOs[i].dcOn);

				}

				else if (uart1RxData[CMD] == arterial[i].rfObjID) {
					arterialIOs[i].rfOn = uart1RxData[VALUE];
					HAL_GPIO_WritePin(arterialIOs[i].rfPort,
							arterialIOs[i].rfPin, arterialIOs[i].rfOn);
				}
			}
			sendDataOnTimeout = true;
			isCmdOk = false;

		} else
			memset(uart1RxData, 0, 20);
		HAL_UART_Receive_IT(&huart1, uart1RxData, BUTTON_DATA_SIZE);

	}
	if (huart == &huart2)
		rxHalRes = HAL_UART_Receive_IT(&huart2, uart2RxData, QUERY_SIZE);

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

void nextionObjectInit() {
	// Initialize arterial 1
	snprintf(arterial[ART1].cObj, sizeof(arterial[ART1].cObj), "%s",
	ARTERIAL1_CURRENT_OBJECT_NAME);
	snprintf(arterial[ART1].cBarObj, sizeof(arterial[ART1].cBarObj), "%s",
	ARTERIAL1_CURRENT_BAR_NAME);
	snprintf(arterial[ART1].dcObjName, sizeof(arterial[ART1].dcObjName), "%s",
	ARTERIAL1_DC_ON_OBJECT_NAME);
	snprintf(arterial[ART1].rfObjName, sizeof(arterial[ART1].rfObjName), "%s",
	ARTERIAL1_RF_ON_OBJECT_NAME);
	arterial[ART1].cObjID = ARTERIAL1_CURRENT_OBJECT_ID;
	arterial[ART1].dcObjID = ARTERIAL1_DC_ON_OBJECT_ID;
	arterial[ART1].rfObjID = ARTERIAL1_RF_ON_OBJECT_ID;

	// Initialize arterial 2
	snprintf(arterial[ART2].cObj, sizeof(arterial[ART2].cObj), "%s",
	ARTERIAL2_CURRENT_OBJECT_NAME);
	snprintf(arterial[ART2].cBarObj, sizeof(arterial[ART2].cBarObj), "%s",
	ARTERIAL2_CURRENT_BAR_NAME);
	snprintf(arterial[ART2].dcObjName, sizeof(arterial[ART2].dcObjName), "%s",
	ARTERIAL2_DC_ON_OBJECT_NAME);
	snprintf(arterial[ART2].rfObjName, sizeof(arterial[ART2].rfObjName), "%s",
	ARTERIAL2_RF_ON_OBJECT_NAME);
	arterial[ART2].cObjID = ARTERIAL2_CURRENT_OBJECT_ID;
	arterial[ART2].dcObjID = ARTERIAL2_DC_ON_OBJECT_ID;
	arterial[ART2].rfObjID = ARTERIAL2_RF_ON_OBJECT_ID;

	// Initialize arterial 3
	snprintf(arterial[ART3].cObj, sizeof(arterial[ART3].cObj), "%s",
	ARTERIAL3_CURRENT_OBJECT_NAME);
	snprintf(arterial[ART3].cBarObj, sizeof(arterial[ART3].cBarObj), "%s",
	ARTERIAL3_CURRENT_BAR_NAME);
	snprintf(arterial[ART3].dcObjName, sizeof(arterial[ART3].dcObjName), "%s",
	ARTERIAL3_DC_ON_OBJECT_NAME);
	snprintf(arterial[ART3].rfObjName, sizeof(arterial[ART3].rfObjName), "%s",
	ARTERIAL3_RF_ON_OBJECT_NAME);
	arterial[ART3].cObjID = ARTERIAL3_CURRENT_OBJECT_ID;
	arterial[ART3].dcObjID = ARTERIAL3_DC_ON_OBJECT_ID;
	arterial[ART3].rfObjID = ARTERIAL3_RF_ON_OBJECT_ID;

	// Initialize arterial 4
	snprintf(arterial[ART4].cObj, sizeof(arterial[ART4].cObj), "%s",
	ARTERIAL4_CURRENT_OBJECT_NAME);
	snprintf(arterial[ART4].cBarObj, sizeof(arterial[ART4].cBarObj), "%s",
	ARTERIAL4_CURRENT_BAR_NAME);
	snprintf(arterial[ART4].dcObjName, sizeof(arterial[ART4].dcObjName), "%s",
	ARTERIAL4_DC_ON_OBJECT_NAME);
	snprintf(arterial[ART4].rfObjName, sizeof(arterial[ART4].rfObjName), "%s",
	ARTERIAL4_RF_ON_OBJECT_NAME);
	arterial[ART4].cObjID = ARTERIAL4_CURRENT_OBJECT_ID;
	arterial[ART4].dcObjID = ARTERIAL4_DC_ON_OBJECT_ID;
	arterial[ART4].rfObjID = ARTERIAL4_RF_ON_OBJECT_ID;

	snprintf(dc12vObjt, sizeof(dc12vObjt), "%s", DC_12V_OBJECT_NAME);
	snprintf(dc24vObjt, sizeof(dc24vObjt), "%s", DC_24V_OBJECT_NAME);
	snprintf(dc48vObjt, sizeof(dc48vObjt), "%s", DC_48V_OBJECT_NAME);
	snprintf(dlBarObj, sizeof(dlBarObj), "%s", DL_BAR_OBJ);
}

void initArterialIOs() {
	for (uint8_t i = 0; i < ARTERIAL_NUMBER; i++) {
		arterialIOs[i].dcOn = false;
		arterialIOs[i].rfOn = false;
		arterialIOs[i].alarm = false;

		if (i == ART1) {
			arterialIOs[i].dcPort = DC_A1_GPIO_Port;
			arterialIOs[i].dcPin = DC_A1_Pin;
			arterialIOs[i].rfPort = RF_A1_GPIO_Port;
			arterialIOs[i].rfPin = RF_A1_Pin;
			arterialIOs[i].AlarmPort = ALARM_A1_GPIO_Port;
			arterialIOs[i].AlarmPin = ALARM_A1_Pin;
		} else if (i == ART2) {
			arterialIOs[i].dcPort = DC_A2_GPIO_Port;
			arterialIOs[i].dcPin = DC_A2_Pin;
			arterialIOs[i].rfPort = RF_A2_GPIO_Port;
			arterialIOs[i].rfPin = RF_A2_Pin;
			arterialIOs[i].AlarmPort = ALARM_A2_GPIO_Port;
			arterialIOs[i].AlarmPin = ALARM_A2_Pin;
		} else if (i == ART3) {
			arterialIOs[i].dcPort = DC_A3_GPIO_Port;
			arterialIOs[i].dcPin = DC_A3_Pin;
			arterialIOs[i].rfPort = RF_A3_GPIO_Port;
			arterialIOs[i].rfPin = RF_A3_Pin;
			arterialIOs[i].AlarmPort = ALARM_A3_GPIO_Port;
			arterialIOs[i].AlarmPin = ALARM_A3_Pin;
		} else if (i == ART4) {
			arterialIOs[i].dcPort = DC_A4_GPIO_Port;
			arterialIOs[i].dcPin = DC_A4_Pin;
			arterialIOs[i].rfPort = RF_A4_GPIO_Port;
			arterialIOs[i].rfPin = RF_A4_Pin;
			arterialIOs[i].AlarmPort = ALARM_A4_GPIO_Port;
			arterialIOs[i].AlarmPin = ALARM_A4_Pin;
		} else {
			// Handle the case for additional indices if required
		}
	}
}

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {
		TIM2->SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag
		sendDataOnTimeout = true;
		//	isRxDataOk = true;
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

float arduino_map(uint16_t value, uint16_t in_min, uint16_t in_max,
		float out_min, float out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void updateAlarm() {
	arterial[ART1].cAlarm = adcValues[ART1_CURRENT_CH] > ART1_THRESHOLD;
	arterial[ART2].cAlarm = adcValues[ART2_CURRENT_CH] > ART2_THRESHOLD;
	arterial[ART3].cAlarm = adcValues[ART3_CURRENT_CH] > ART3_THRESHOLD;
	arterial[ART4].cAlarm = adcValues[ART4_CURRENT_CH] > ART4_THRESHOLD;
	for (int i = 0; i < ARTERIAL_NUMBER; i++)
		arterialIOs[i].alarm = HAL_GPIO_ReadPin(arterialIOs[i].AlarmPort,
				arterialIOs[i].AlarmPin) == GPIO_PIN_SET;

	pinDownlinkAlarm = HAL_GPIO_ReadPin(ALARM_VIN_GPIO_Port,
	ALARM_VIN_Pin == GPIO_PIN_SET);	//ESTADO ALARM5 fuente >
	vAlarm = adcValues[VOLTAGE_CH] > VOLT_THRESHOLD;
	dlAlarm = adcValues[DOWNLINK_LVL_CH] > DOWNLINK_THRESHOLD;
	ulAlarm = adcValues[UPLINK_LVL_CH] > UPLINK_THRESHOLD;
}

void sendDataIfTimeout() {

	if (sendDataOnTimeout == false)
		return;
	sendDataOnTimeout = false;
	bool dcOn = false;
	bool rfOn = false;
	uint32_t currFontColor;
	uint32_t fontColor;
	uint32_t backColor;

	for (int i = 0; i < ARTERIAL_NUMBER; i++) {

		floatToNextion(arterial[i].cObj, arterial[i].current, 2);
		intToNextion(arterial[i].cBarObj, arterial[i].cBar);
		dcOn |= arterialIOs[i].dcOn;
		rfOn |= arterialIOs[i].rfOn;

		if (arterialIOs[i].dcOn) {
			arterialIOs[i].alarm = HAL_GPIO_ReadPin(arterialIOs[i].AlarmPort,
					arterialIOs[i].AlarmPin) == GPIO_PIN_SET;
			if (arterialIOs[i].alarm == false) {

				if (arterial[i].dcAlarmColor == ORANGE_C)
					arterial[i].dcAlarmColor = RED_C;
				else if (arterial[i].dcAlarmColor == RED_C)
					arterial[i].dcAlarmColor = ORANGE_C;
				else
					arterial[i].dcAlarmColor = RED_C;
			} else
				arterial[i].dcAlarmColor = ORANGE_C;
		} else
			arterial[i].dcAlarmColor = GREY_C;
		colorToNextionOn(arterial[i].dcObjName, arterial[i].dcAlarmColor);

		if (arterialIOs[i].dcOn) {
			if (arterial[i].cAlarm == true) {

				if (arterial[i].cAlarmColor == WHITE_C)
					arterial[i].cAlarmColor = RED_C;
				else if (arterial[i].cAlarmColor == RED_C)
					arterial[i].cAlarmColor = WHITE_C;
				else
					arterial[i].cAlarmColor = RED_C;
			} else
				arterial[i].cAlarmColor = WHITE_C;
		} else
			arterial[i].cAlarmColor = GREY_C;
		colorToNextion(arterial[i].cObj, arterial[i].cAlarmColor);

		fontColor = arterialIOs[i].dcOn ? BLACK_C : WHITE_C;
//		backColor = arterialIOs[i].dcOn ? WHITE_C : GREY_C;
//		colorToNextion(arterial[i].cObj,
//				arterial[i].cAlarm ? RED_C : backColor);
		textColorToNextion(arterial[i].cObj,
				arterial[i].cAlarm ? BLACK_C : fontColor);

	}

	fontColor = dcOn ? BLACK_C : WHITE_C;
	backColor = dcOn ? WHITE_C : GREY_C;
	textColorToNextion(dc12vObjt, fontColor);
	textColorToNextion(dc24vObjt, fontColor);
	textColorToNextion(dc48vObjt, fontColor);

	if (dcState == VIN_12V) {
		colorToNextion(dc12vObjt, ORANGE_C);
		colorToNextion(dc24vObjt, backColor);
		colorToNextion(dc48vObjt, backColor);
	} else if (dcState == VIN_24V) {
		colorToNextion(dc12vObjt, backColor);
		colorToNextion(dc24vObjt, ORANGE_C);
		colorToNextion(dc48vObjt, backColor);

	} else if (dcState == VIN_48V) {
		colorToNextion(dc12vObjt, backColor);
		colorToNextion(dc24vObjt, backColor);
		colorToNextion(dc48vObjt, ORANGE_C);
	}

	fontColor = rfOn ? BLACK_C : WHITE_C;
	backColor = rfOn ? WHITE_C : GREY_C;
	floatToNextion(DOWNLINK_OBJECT_NAME, downlinkLevel, 1);
	colorToNextion(DOWNLINK_OBJECT_NAME, dlAlarm ? RED_C : backColor);
	textColorToNextion(DOWNLINK_OBJECT_NAME, dlAlarm ? BLACK_C : fontColor);
	intToNextion(dlBarObj, dlBar);
	intToNextion(DC_BAR_OBJ, dcBar);

}

void updateDlValues() {
	downlinkLevel = arduino_map(adcValues[DOWNLINK_LVL_CH],
	ADC_DOWNLINK_MIN,
	ADC_DOWNLINK_MAX, DOWNLINK_LEVEL_MIN, DOWNLINK_LEVEL_MAX);

	dlBar = arduino_map(adcValues[DOWNLINK_LVL_CH], 370, 1700, 0, 100);
	if ((int) dlBar < 0)
		dlBar = 0;

	if ((int) dlBar > 100)
		dlBar = 100;
}

void updateDcValues() {

	dcValue = arduino_map(adcValues[VOLTAGE_CH],
	ADC_MIN_VALUE, ADC_MAX_VALUE, VOLTAGE_MIN, VOLTAGE_MAX);

	dcState =
			(adcValues[VOLTAGE_CH] >= VOLTAGE_THRESHOLD_5V
					&& adcValues[VOLTAGE_CH] < VOLTAGE_THRESHOLD_18V) ?
					VIN_12V :
			(adcValues[VOLTAGE_CH] >= VOLTAGE_THRESHOLD_18V
					&& adcValues[VOLTAGE_CH] < VOLTAGE_THRESHOLD_36V) ?
					VIN_24V :
			(adcValues[VOLTAGE_CH] >= VOLTAGE_THRESHOLD_36V) ?
					VIN_48V : VIN_UNKNOW;

	dcBar = arduino_map(dcValue, 0, 50, 0, 100);
	if ((int) dcBar < 0)
		dcBar = 0;
	if ((int) dcBar > 100)
		dcBar = 100;
}

uint8_t dataSize = 0;

uint8_t updateUART2Tx() {
	uint8_t idx = 0;
	uart2TxData[idx++] = RDSS_START_MARK;
	uart2TxData[idx++] = HEAD_END;
	uart2TxData[idx++] = 0x00;
	uart2TxData[idx++] = QUERY_STATUS;
	uart2TxData[idx++] = 0x00;
	uart2TxData[idx++] = dataSize;
	dataSize = idx; // 6
	uart2TxData[idx] = arterialIOs[0].dcOn | (arterialIOs[0].rfOn << 1);
	uart2TxData[idx] |= arterialIOs[1].dcOn << 2 | arterialIOs[1].rfOn << 3;
	uart2TxData[idx] |= arterialIOs[2].dcOn << 4 | arterialIOs[2].rfOn << 5;
	uart2TxData[idx] |= arterialIOs[3].dcOn << 6 | arterialIOs[3].rfOn << 7;
	idx++;
	uart2TxData[idx++] = adcValues[ART1_CURRENT_CH];
	uart2TxData[idx++] = adcValues[ART1_CURRENT_CH] >> 8;
	uart2TxData[idx++] = adcValues[ART2_CURRENT_CH];
	uart2TxData[idx++] = adcValues[ART2_CURRENT_CH] >> 8;
	uart2TxData[idx++] = adcValues[ART3_CURRENT_CH];
	uart2TxData[idx++] = adcValues[ART3_CURRENT_CH] >> 8;
	uart2TxData[idx++] = adcValues[ART4_CURRENT_CH];
	uart2TxData[idx++] = adcValues[ART4_CURRENT_CH] >> 8;
	uart2TxData[idx++] = adcValues[VOLTAGE_CH];
	uart2TxData[idx++] = adcValues[VOLTAGE_CH] >> 8;
	uart2TxData[idx++] = adcValues[DOWNLINK_LVL_CH];
	uart2TxData[idx++] = adcValues[DOWNLINK_LVL_CH] >> 8;
	uart2TxData[idx++] = adcValues[UPLINK_LVL_CH];
	uart2TxData[idx++] = adcValues[UPLINK_LVL_CH] >> 8;
	dataSize = idx - dataSize; //21 - 6 = 15
	setCrc(uart2TxData, idx);
	uart2TxData[idx + 2] = RDSS_END_MARK;

	return (idx + 3);
}

void updateArterialValues() {
	for (int i = 0; i < ARTERIAL_NUMBER; i++) {

		int adcIndex = arterialToADCIndex[i];
		arterial[i].current = arduino_map(adcValues[adcIndex],
		ADC_CURRENT_MIN_VALUE,
		ADC_CURRENT_MAX_VALUE, CURRENT_MIN_VALUE, CURRENT_MAX_VALUE);
		arterial[i].cBar = arduino_map(adcValues[adcIndex], 0, 1300, 0, 100);
		if ((int) arterial[i].cBar > 100)
			arterial[i].cBar = 100;
		if ((int) arterial[i].cBar < 0)
			arterial[i].cBar = 0;
		if (adcValues[adcIndex] < 1)
			arterial[i].current = 0;
	}
}

void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_UIF) {
		TIM3->SR &= ~TIM_SR_UIF;  // Clear the update interrupt flag
	}

	for (int adcIdx = 0; adcIdx < ADC_CHANNELS; adcIdx++) {
		// Subtract oldest value from the sum

		adcSum[adcIdx] -= adcReadings[adcIdx][adcCounter[adcIdx]];
		// Subtraction overflow check
		if ((int32_t) adcSum[adcIdx] < 0)
			adcSum[adcIdx] = 0;

		// Store the new value in the buffer
		adcReadings[adcIdx][adcCounter[adcIdx]] = adcValues[adcIdx];

		// Add new value to the sumh
		adcSum[adcIdx] += adcValues[adcIdx];

		// Calculate and return the average
		adcMA[adcIdx] = (adcSum[adcIdx] / ADC_WINDOW_SIZE);

		// Increment the current index and wrap around if necessary
		adcCounter[adcIdx]++;
		if (adcCounter[adcIdx] >= ADC_WINDOW_SIZE) {
			adcCounter[adcIdx] = 0;
		}

	}

}

void startTimer3(void) {
	TIM3->CR1 |= TIM_CR1_CEN;  // Start the timer
}

void configureTimer2() {
	//  MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	// Enable the timer clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	// Configure the timer
	TIM2->PSC = 7999; // Prescaler value for a 1 ms time base (assuming a 48 MHz clock)
	TIM2->ARR = 499; // Auto-reload value for a 500 ms period
	//	TIM2->ARR = 3999;  // Auto-reload value for a 500 ms period
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->CR1 |= TIM_CR1_URS;
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM2_IRQn, 0); // Set the interrupt priority
	NVIC_EnableIRQ(TIM2_IRQn); // Enable the interrupt
}

void configureTimer3() {
	// Enable the timer clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	// Configure the timer
	TIM3->PSC = 3999; // Prescaler value for a 1 ms time base (assuming a 48 MHz clock)
	TIM3->ARR = 199; // Auto-reload value for a 500 ms period
	//	TIM2->ARR = 3999;  // Auto-reload value for a 500 ms period
	TIM3->CR1 |= TIM_CR1_ARPE;
	TIM3->CR1 |= TIM_CR1_URS;
	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM3_IRQn, 0); // Set the interrupt priority
	NVIC_EnableIRQ(TIM3_IRQn); // Enable the interrupt
}
uint32_t startTicks = 0;

void processRs485Cmd() {

	uart2TxIndex = updateUART2Tx();

	if (checkModuleValidity(uart2RxData, QUERY_SIZE) != VALID_MODULE)
		return;
	if (checkCRCValidity(uart2RxData, QUERY_SIZE) != DATA_OK)
		return;

	if (uart2RxData[CMD_INDEX] == QUERY_STATUS) {
		HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart2, uart2TxData, uart2TxIndex,
		HAL_MAX_DELAY);
		HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
	}
	memset(uart2RxData, 0, UART2_RX_BUFFLEN);
	uart2RxSize = 0;
}

void uartReinit(uint16_t timeout) {
	if (HAL_GetTick() - startTicks > timeout) {
		HAL_UART_DeInit(&huart1);
		HAL_Delay(1);
		MX_USART1_UART_Init();
		HAL_UART_Init(&huart1); // Enable USART2

		HAL_UART_Receive_IT(&huart1, uart1RxData, BUTTON_DATA_SIZE);
		HAL_UART_DeInit(&huart2);
		HAL_Delay(1);
		MX_USART2_UART_Init();
		HAL_UART_Init(&huart2); // Enable USART2
		HAL_UART_Receive_IT(&huart2, uart2RxData, QUERY_SIZE);
		startTicks = HAL_GetTick();
	}
}

uint32_t keepAliveStartTicks;

void enableKeepAliveLed() {
	if (HAL_GetTick() - keepAliveStartTicks > 1000) {
		keepAliveStartTicks = HAL_GetTick();
		HAL_GPIO_WritePin(KA_LED_GPIO_Port, KA_LED_Pin, GPIO_PIN_SET);
	} else if (HAL_GetTick() - keepAliveStartTicks > 50)
		HAL_GPIO_WritePin(KA_LED_GPIO_Port, KA_LED_Pin, GPIO_PIN_RESET);
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
//	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */

	// Initialize necessary peripherals and configurations
	res = HAL_ADCEx_Calibration_Start(&hadc);
	if (res != HAL_OK)
		Error_Handler();

	res = HAL_ADC_Start_DMA(&hadc, (uint32_t*) adcValues, ADC_CHANNELS);
	if (res != HAL_OK)
		Error_Handler();

	res = HAL_UART_Receive_IT(&huart1, uart1RxData, BUTTON_DATA_SIZE);
	if (res != HAL_OK)
		Error_Handler();

	res = HAL_UART_Receive_IT(&huart2, uart2RxData, QUERY_SIZE);
	if (res != HAL_OK)
		Error_Handler();

	nextionObjectInit();
	initArterialIOs();

	configureTimer2();
	configureTimer3();
	startTimer2();
	startTimer3();

//	HAL_IWDG_Refresh(&hiwdg);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		uartReinit(5000);
		processRs485Cmd();
		updateArterialValues();
		updateDlValues();
		updateDcValues();
		updateAlarm();
		enableKeepAliveLed();
		sendDataIfTimeout();
//		HAL_IWDG_Refresh(&hiwdg);

	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

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
			| RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 1250;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

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
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
	DE_Pin | DC_A1_Pin | DC_A2_Pin | DC_A3_Pin | DC_A4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			RF_A4_Pin | RF_A3_Pin | RF_A2_Pin | KA_LED_Pin | NEXTION_LED_Pin
					| DATA_OK_Pin | ADC_CPLT_Pin | RF_A1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : DE_Pin DC_A1_Pin DC_A2_Pin DC_A3_Pin
	 DC_A4_Pin */
	GPIO_InitStruct.Pin =
	DE_Pin | DC_A1_Pin | DC_A2_Pin | DC_A3_Pin | DC_A4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RF_A4_Pin RF_A3_Pin RF_A2_Pin KA_LED_Pin
	 NEXTION_LED_Pin DATA_OK_Pin ADC_CPLT_Pin RF_A1_Pin */
	GPIO_InitStruct.Pin = RF_A4_Pin | RF_A3_Pin | RF_A2_Pin | KA_LED_Pin
			| NEXTION_LED_Pin | DATA_OK_Pin | ADC_CPLT_Pin | RF_A1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB11 PB4 PB5
	 PB6 PB7 PB8 PB9 */
	GPIO_InitStruct.Pin =
	GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
			| GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : ALARM_A4_Pin ALARM_A3_Pin ALARM_A2_Pin ALARM_A1_Pin
	 ALARM_VIN_Pin */
	GPIO_InitStruct.Pin = ALARM_A4_Pin | ALARM_A3_Pin | ALARM_A2_Pin
			| ALARM_A1_Pin | ALARM_VIN_Pin;
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
