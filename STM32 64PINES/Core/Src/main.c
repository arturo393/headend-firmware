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

#define ADC_DOWNLINK_MAX 1634
#define DOWNLINK_LEVEL_MAX 11.7
#define ADC_DOWNLINK_MIN 1014
#define DOWNLINK_LEVEL_MIN -7.8
#define DOWNLINK_THRESHOLD 1600

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

#define YELLOW_C 65504
#define RED_C 63488
#define WHITE_C 65535
#define GREEN_ON_C 2016
#define GREEN_BACKGROUND 1024
#define GREEN_DARK_BACKGROUND 480
#define GREY_BACKGROUND 50712
#define ORANGE_C 64128
#define GREY_C 21162
#define BLACK_C 0

#define SEND_DATA_TO_NEXION_TIMEOUT 100
#define ADC_WINDOW_SIZE 50

typedef enum {
	VIN_0V, VIN_12V, VIN_24V, VIN_48V, VIN_UNKNOW
} VOLTAGE_STATE_T;

int uplinkLevel;
float downlinkLevel;
uint8_t dc_bar;
uint8_t dlBar;
char dlBarObj[4];
bool vAlarm = false;
bool dlAlarm = false;
bool ulAlarm = false;
GPIO_PinState pinDcAlarm;
GPIO_PinState pinDownlinkAlarm;
char dcValueObjName[4];
char dc_12v_object[4];
char dc_24v_object[4];
char dc_48v_object[4];
char dcBarObjt[4];
VOLTAGE_STATE_T dc_voltage_state;
float dc_voltage_value;
bool arterial_change[10] = { false };
uint8_t arterial_change_times = 0;
bool arterial_update = false;
typedef enum {
	ART1, ART2, ART3, ART4, ARTERIAL_NUMBER
} ARTERIAL_NUM_T;

typedef enum NEXTION_UART_INDEX {
	START, CMD, VALUE, END, BUTTON_DATA_SIZE
} NEXTION_INDEX_T;

typedef enum {
	ART1_CURRENT_CHANNEL,
	ART4_CURRENT_CH,
	VOLTAGE_CHANNEL,
	DOWNLINK_LVL_CH,
	UPLINK_LVL_CH,
	ART3_CURRENT_CH,
	ART2_CURRENT_CHANNEL,
	ADC_CHANNELS
} ADC_CHANNEL_T;

typedef struct ARTERIAL {

	float current;
	uint8_t cBar;
	char cObj[4];
	char cBarObj[4];
	char rfObjName[4];
	char dc_object_name[4];
	uint8_t vObjID;
	uint8_t cObjID;
	uint8_t pObjID;
	uint8_t rfObjID;
	uint8_t dcObjID;
	bool current_alarm;
	uint32_t dc_alarm_color;
	uint32_t cAlarmColor;
} Arterial;

// Create a lookup table to map arterial numbers to ADC indices
const int arterialToADCIndex[ARTERIAL_NUMBER] = { ART1_CURRENT_CHANNEL,
		ART2_CURRENT_CHANNEL, ART3_CURRENT_CH, ART4_CURRENT_CH };

typedef struct {
	GPIO_TypeDef *dcPort;
	uint16_t dcPin;
	GPIO_TypeDef *rfPort;
	uint16_t rfPin;
	GPIO_TypeDef *alarm_port;
	uint16_t alarm_pin;
	bool dc_on;
	bool dcOnLast;
	bool rfOn;
	bool dc_alarm;
} ArterialIO;

Arterial arterial[ARTERIAL_NUMBER];
ArterialIO arterial_io[ARTERIAL_NUMBER];

// UART1 variables for nextion comunication
uint8_t uartBuffer[20];
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

void set_arterial_output_on(ArterialIO arterial_io) {
	uint8_t pulse_width = 0;
	uint16_t pulse_times = 0;
	uint16_t pulse_period = 0;
	switch (dc_voltage_state) {

	case VIN_0V:
		pulse_width = 0;
		pulse_times = 0;
		pulse_period = 0;
		break;
	case VIN_12V:
		pulse_width = 39;
		pulse_times = 100;
		pulse_period = 100;
		break;
	case VIN_24V:
		pulse_width = 15;
		pulse_times = 45;
		pulse_period = 50;
		break;
	case VIN_48V:
		pulse_width = 10;
		pulse_times = 100;
		pulse_period = 33;
		break;
	default:
		pulse_width = 0;
		pulse_times = 0;
		pulse_period = 0;
		break;
	}

	for (uint16_t i = 0; i < pulse_times; i++) {
		for (uint8_t j = 0; j < pulse_period; j++) {
			if (j < pulse_width)
				HAL_GPIO_WritePin(arterial_io.dcPort, arterial_io.dcPin,
						GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(arterial_io.dcPort, arterial_io.dcPin,
						GPIO_PIN_RESET);
		}
	}
	HAL_GPIO_WritePin(arterial_io.dcPort, arterial_io.dcPin, GPIO_PIN_SET);
}

void set_arterial_output_off(ArterialIO arterial_io) {
	uint8_t pulse_width = 0;
	uint16_t pulse_times = 0;
	uint16_t pulse_period = 0;
	uint8_t count = 0;
	uint8_t set = 0;
	switch (dc_voltage_state) {

	case VIN_0V:
		pulse_width = 0;
		pulse_times = 0;
		pulse_period = 0;
		set = 0;
		break;
	case VIN_12V:
		pulse_width = 0;
		pulse_times = 0;
		pulse_period = 0;
		set = 0;
		break;
	case VIN_24V:
		pulse_width = 29;
		pulse_times = 37;
		pulse_period = 100;
		set = 20;
		break;
	case VIN_48V:
		pulse_width = 29;
		pulse_times = 40;
		pulse_period = 100;
		set = 30;
		break;
	default:
		pulse_width = 0;
		pulse_times = 0;
		pulse_period = 0;
		set = 0;
		break;
	}

	for (uint16_t i = 0; i < pulse_times; i++) {
		if(pulse_times > set){
			count = 1;
		}
		for (uint8_t j = 0; j < pulse_period; j++) {
			if (j < (pulse_width-count))
				HAL_GPIO_WritePin(arterial_io.dcPort, arterial_io.dcPin,
						GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(arterial_io.dcPort, arterial_io.dcPin,
						GPIO_PIN_RESET);
		}
	}
	HAL_GPIO_WritePin(arterial_io.dcPort, arterial_io.dcPin, GPIO_PIN_RESET);
	count = 0;
}

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

void set_color_to_nextion(char *obj, uint32_t color) {
	// convert to the integer
	uint8_t len = 0;
	uint8_t buffer[30] = { 0 };
	len = sprintf((char*) buffer, "%s.bco=%lu", obj, color);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
}

void set_on_color_to_nextion(char *obj, uint32_t color) {
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

void set_text_color_to_nextion(char *obj, uint32_t color) {
	// convert to the integer
	uint8_t len = 0;
	uint8_t buffer[30] = { 0 };
	len = sprintf((char*) buffer, "%s.pco=%lu", obj, color);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
}

void sendArterialToNextion(Arterial arterial) {
	floatToNextion(arterial.cObj, arterial.current, 2);
	if (arterial.current_alarm)
		set_color_to_nextion(arterial.cObj, RED_C);
	else
		set_color_to_nextion(arterial.cObj, WHITE_C);

}

HAL_StatusTypeDef rxHalRes;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		if ((uartBuffer[START] == *START_RX_DATA)
				&& (uartBuffer[END] == *END_RX_DATA)) {

			for (uint8_t i = 0; i < ARTERIAL_NUMBER; i++) {
				if (uartBuffer[CMD] == arterial[i].dcObjID) {
					arterial_io[i].dc_on = uartBuffer[VALUE];
					if (arterial_io[i].dc_on)
						set_arterial_output_on(arterial_io[i]);
					else
						set_arterial_output_off(arterial_io[i]);
				} else if (uartBuffer[CMD] == arterial[i].rfObjID) {
					arterial_io[i].rfOn = uartBuffer[VALUE];
					HAL_GPIO_WritePin(arterial_io[i].rfPort,
							arterial_io[i].rfPin, arterial_io[i].rfOn);
				}
			}
			isCmdOk = false;
		} else
			memset(uartBuffer, 0, 20);
		HAL_UART_Receive_IT(&huart1, uartBuffer, BUTTON_DATA_SIZE);
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
	snprintf(arterial[ART1].dc_object_name,
			sizeof(arterial[ART1].dc_object_name), "%s",
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
	snprintf(arterial[ART2].dc_object_name,
			sizeof(arterial[ART2].dc_object_name), "%s",
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
	snprintf(arterial[ART3].dc_object_name,
			sizeof(arterial[ART3].dc_object_name), "%s",
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
	snprintf(arterial[ART4].dc_object_name,
			sizeof(arterial[ART4].dc_object_name), "%s",
			ARTERIAL4_DC_ON_OBJECT_NAME);
	snprintf(arterial[ART4].rfObjName, sizeof(arterial[ART4].rfObjName), "%s",
	ARTERIAL4_RF_ON_OBJECT_NAME);
	arterial[ART4].cObjID = ARTERIAL4_CURRENT_OBJECT_ID;
	arterial[ART4].dcObjID = ARTERIAL4_DC_ON_OBJECT_ID;
	arterial[ART4].rfObjID = ARTERIAL4_RF_ON_OBJECT_ID;

	snprintf(dc_12v_object, sizeof(dc_12v_object), "%s", DC_12V_OBJECT_NAME);
	snprintf(dc_24v_object, sizeof(dc_24v_object), "%s", DC_24V_OBJECT_NAME);
	snprintf(dc_48v_object, sizeof(dc_48v_object), "%s", DC_48V_OBJECT_NAME);
	snprintf(dlBarObj, sizeof(dlBarObj), "%s", DL_BAR_OBJ);
}

void initArterialIOs() {
	for (uint8_t i = 0; i < ARTERIAL_NUMBER; i++) {
		arterial_io[i].dc_on = false;
		arterial_io[i].rfOn = false;
		arterial_io[i].dc_alarm = false;

		if (i == ART1) {
			arterial_io[i].dcPort = DC_A1_GPIO_Port;
			arterial_io[i].dcPin = DC_A1_Pin;
			arterial_io[i].rfPort = RF_A1_GPIO_Port;
			arterial_io[i].rfPin = RF_A1_Pin;
			arterial_io[i].alarm_port = ALARM_A1_GPIO_Port;
			arterial_io[i].alarm_pin = ALARM_A1_Pin;
		} else if (i == ART2) {
			arterial_io[i].dcPort = DC_A2_GPIO_Port;
			arterial_io[i].dcPin = DC_A2_Pin;
			arterial_io[i].rfPort = RF_A2_GPIO_Port;
			arterial_io[i].rfPin = RF_A2_Pin;
			arterial_io[i].alarm_port = ALARM_A2_GPIO_Port;
			arterial_io[i].alarm_pin = ALARM_A2_Pin;
		} else if (i == ART3) {
			arterial_io[i].dcPort = DC_A3_GPIO_Port;
			arterial_io[i].dcPin = DC_A3_Pin;
			arterial_io[i].rfPort = RF_A3_GPIO_Port;
			arterial_io[i].rfPin = RF_A3_Pin;
			arterial_io[i].alarm_port = ALARM_A3_GPIO_Port;
			arterial_io[i].alarm_pin = ALARM_A3_Pin;
		} else if (i == ART4) {
			arterial_io[i].dcPort = DC_A4_GPIO_Port;
			arterial_io[i].dcPin = DC_A4_Pin;
			arterial_io[i].rfPort = RF_A4_GPIO_Port;
			arterial_io[i].rfPin = RF_A4_Pin;
			arterial_io[i].alarm_port = ALARM_A4_GPIO_Port;
			arterial_io[i].alarm_pin = ALARM_A4_Pin;
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
	arterial[ART1].current_alarm = adcMA[ART1_CURRENT_CHANNEL] > ART1_THRESHOLD;
	arterial[ART2].current_alarm = adcMA[ART2_CURRENT_CHANNEL] > ART2_THRESHOLD;
	arterial[ART3].current_alarm = adcMA[ART3_CURRENT_CH] > ART3_THRESHOLD;
	arterial[ART4].current_alarm = adcMA[ART4_CURRENT_CH] > ART4_THRESHOLD;
	for (int i = 0; i < ARTERIAL_NUMBER; i++)
		arterial_io[i].dc_alarm = HAL_GPIO_ReadPin(arterial_io[i].alarm_port,
				arterial_io[i].alarm_pin) == GPIO_PIN_RESET;

	pinDownlinkAlarm = HAL_GPIO_ReadPin(ALARM_VIN_GPIO_Port,
	ALARM_VIN_Pin == GPIO_PIN_SET);	//ESTADO ALARM5 fuente >
	vAlarm = adcValues[VOLTAGE_CHANNEL] > VOLT_THRESHOLD;
	dlAlarm = adcValues[DOWNLINK_LVL_CH] > DOWNLINK_THRESHOLD;
	ulAlarm = adcValues[UPLINK_LVL_CH] > UPLINK_THRESHOLD;
}

void update_dc_voltage_display(bool is_dc_on) {
	uint32_t font_color = is_dc_on ? BLACK_C : WHITE_C;
	uint32_t background_color = is_dc_on ? WHITE_C : GREY_C;

	/* Set text colors for all voltage objects */
	set_text_color_to_nextion(dc_12v_object, font_color);
	set_text_color_to_nextion(dc_24v_object, font_color);
	set_text_color_to_nextion(dc_48v_object, font_color);

	/* Set background colors and highlight the active voltage object */
	switch (dc_voltage_state) {
	case VIN_12V:
		set_color_to_nextion(dc_12v_object, GREEN_ON_C);
		set_color_to_nextion(dc_24v_object, background_color);
		set_color_to_nextion(dc_48v_object, background_color);
		break;
	case VIN_24V:
		set_color_to_nextion(dc_12v_object, background_color);
		set_color_to_nextion(dc_24v_object, GREEN_ON_C);
		set_color_to_nextion(dc_48v_object, background_color);
		break;
	case VIN_48V:
		set_color_to_nextion(dc_12v_object, background_color);
		set_color_to_nextion(dc_24v_object, background_color);
		set_color_to_nextion(dc_48v_object, GREEN_ON_C);
		break;
	case VIN_0V:
		set_color_to_nextion(dc_12v_object, background_color);
		set_color_to_nextion(dc_24v_object, background_color);
		set_color_to_nextion(dc_48v_object, background_color);
		break;
	default: /* Handle other voltage states or unknown states */
		set_color_to_nextion(dc_12v_object, background_color);
		set_color_to_nextion(dc_24v_object, background_color);
		set_color_to_nextion(dc_48v_object, background_color);
		break;
	}
}

void set_dc_button_alarm_color(uint8_t arterial_index) {

	if (!arterial_io[arterial_index].dc_on) {
		arterial[arterial_index].dc_alarm_color = GREY_C;
		set_on_color_to_nextion(arterial[arterial_index].dc_object_name,
				arterial[arterial_index].dc_alarm_color);
		return;
	}

	//arterial_io[arterial_index].dc_alarm = HAL_GPIO_ReadPin(
	//		arterial_io[arterial_index].alarm_port,
	//		arterial_io[arterial_index].alarm_pin) == GPIO_PIN_SET;

	if (arterial_io[arterial_index].dc_alarm == false) {
		arterial[arterial_index].dc_alarm_color = GREEN_ON_C;
		set_on_color_to_nextion(arterial[arterial_index].dc_object_name,
				arterial[arterial_index].dc_alarm_color);
		return;
	}

	if (arterial[arterial_index].dc_alarm_color == GREEN_ON_C)
		arterial[arterial_index].dc_alarm_color = RED_C;
	else if (arterial[arterial_index].dc_alarm_color == RED_C)
		arterial[arterial_index].dc_alarm_color = GREEN_ON_C;
	else
		arterial[arterial_index].dc_alarm_color = GREEN_ON_C;

	set_on_color_to_nextion(arterial[arterial_index].dc_object_name,
			arterial[arterial_index].dc_alarm_color);
}

void send_data_to_nextion_on_timeout() {

	bool dcOn = false;
	bool rfOn = false;
	uint32_t fontColor;
	uint32_t backColor;

	for (int i = 0; i < ARTERIAL_NUMBER; i++) {

		floatToNextion(arterial[i].cObj, arterial[i].current, 2);
		intToNextion(arterial[i].cBarObj, arterial[i].cBar);
		dcOn |= arterial_io[i].dc_on;
		rfOn |= arterial_io[i].rfOn;

		set_dc_button_alarm_color(i);

		if (arterial_io[i].dc_on) {
			if (arterial[i].current_alarm == true) {
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
		set_color_to_nextion(arterial[i].cObj, arterial[i].cAlarmColor);

		fontColor = arterial_io[i].dc_on ? BLACK_C : WHITE_C;
//		backColor = arterialIOs[i].dcOn ? WHITE_C : GREY_C;
//		colorToNextion(arterial[i].cObj,
//				arterial[i].cAlarm ? RED_C : backColor);
		set_text_color_to_nextion(arterial[i].cObj,
				arterial[i].current_alarm ? BLACK_C : fontColor);

	}

	update_dc_voltage_display(dcOn);

	fontColor = rfOn ? BLACK_C : WHITE_C;
	backColor = rfOn ? WHITE_C : GREY_C;

	if (downlinkLevel <= -99.9) {
		fontColor = RED_C;
		backColor = RED_C;
	}
	floatToNextion(DOWNLINK_OBJECT_NAME, downlinkLevel, 1);
	set_color_to_nextion(DOWNLINK_OBJECT_NAME, dlAlarm ? RED_C : backColor);
	set_text_color_to_nextion(DOWNLINK_OBJECT_NAME,
			dlAlarm ? BLACK_C : fontColor);
	intToNextion(dlBarObj, dlBar);
	intToNextion(DC_BAR_OBJ, dc_bar);

}

void updateDlValues() {
	downlinkLevel = arduino_map(adcValues[DOWNLINK_LVL_CH],
	ADC_DOWNLINK_MIN,
	ADC_DOWNLINK_MAX, DOWNLINK_LEVEL_MIN, DOWNLINK_LEVEL_MAX);
	if (adcValues[DOWNLINK_LVL_CH] < 380)
		downlinkLevel = -99.9;

	dlBar = arduino_map(adcValues[DOWNLINK_LVL_CH], 370, 1700, 0, 100);
	if ((int) dlBar < 0)
		dlBar = 0;

	if ((int) dlBar > 100)
		dlBar = 100;
}

void update_dc_voltage_values() {

	/* Read ADC value */
	uint16_t adc_value = adcValues[VOLTAGE_CHANNEL];

	/* Calculate DC voltage */
	dc_voltage_value = arduino_map(adc_value,
	ADC_MIN_VALUE, ADC_MAX_VALUE, VOLTAGE_MIN, VOLTAGE_MAX);

	/* Determine DC state based on voltage ranges */
	if (adc_value < VOLTAGE_THRESHOLD_5V)
		dc_voltage_state = VIN_0V;
	else if (adc_value >= VOLTAGE_THRESHOLD_5V
			&& adc_value < VOLTAGE_THRESHOLD_18V)
		dc_voltage_state = VIN_12V;
	else if (adc_value >= VOLTAGE_THRESHOLD_18V
			&& adc_value < VOLTAGE_THRESHOLD_36V)
		dc_voltage_state = VIN_24V;
	else if (adc_value >= VOLTAGE_THRESHOLD_36V)
		dc_voltage_state = VIN_48V;
	else
		dc_voltage_state = VIN_UNKNOW;

	dc_bar = arduino_map(dc_voltage_value, 0, 50, 0, 100);
	if ((int) dc_bar < 0)
		dc_bar = 0;
	if ((int) dc_bar > 100)
		dc_bar = 100;
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
	uart2TxData[idx] = arterial_io[0].dc_on | (arterial_io[0].rfOn << 1);
	uart2TxData[idx] |= arterial_io[1].dc_on << 2 | arterial_io[1].rfOn << 3;
	uart2TxData[idx] |= arterial_io[2].dc_on << 4 | arterial_io[2].rfOn << 5;
	uart2TxData[idx] |= arterial_io[3].dc_on << 6 | arterial_io[3].rfOn << 7;
	idx++;
	uart2TxData[idx++] = adcValues[ART1_CURRENT_CHANNEL];
	uart2TxData[idx++] = adcValues[ART1_CURRENT_CHANNEL] >> 8;
	uart2TxData[idx++] = adcValues[ART2_CURRENT_CHANNEL];
	uart2TxData[idx++] = adcValues[ART2_CURRENT_CHANNEL] >> 8;
	uart2TxData[idx++] = adcValues[ART3_CURRENT_CH];
	uart2TxData[idx++] = adcValues[ART3_CURRENT_CH] >> 8;
	uart2TxData[idx++] = adcValues[ART4_CURRENT_CH];
	uart2TxData[idx++] = adcValues[ART4_CURRENT_CH] >> 8;
	uart2TxData[idx++] = adcValues[VOLTAGE_CHANNEL];
	uart2TxData[idx++] = adcValues[VOLTAGE_CHANNEL] >> 8;
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
		arterial[i].current = arduino_map(adcMA[adcIndex],
		ADC_CURRENT_MIN_VALUE,
		ADC_CURRENT_MAX_VALUE, CURRENT_MIN_VALUE, CURRENT_MAX_VALUE);
		if (arterial[i].current > 1) {
			uint8_t j = 0;
			j++;
		}
		arterial[i].cBar = arduino_map(adcMA[adcIndex], 0, 1300, 0, 100);
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
uint32_t sendDataToNextionTicks = 0;
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

		HAL_UART_Receive_IT(&huart1, uartBuffer, BUTTON_DATA_SIZE);
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

uint32_t maxValue = 0;
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
	MX_DMA_Init();
	MX_ADC_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */

	// Initialize necessary peripherals and configurations
	HAL_StatusTypeDef res;
	res = HAL_ADCEx_Calibration_Start(&hadc);
	if (res != HAL_OK)
		Error_Handler();

	res = HAL_ADC_Start_DMA(&hadc, (uint32_t*) adcValues, ADC_CHANNELS);
	if (res != HAL_OK)
		Error_Handler();

	res = HAL_UART_Receive_IT(&huart1, uartBuffer, BUTTON_DATA_SIZE);
	if (res != HAL_OK)
		Error_Handler();

	res = HAL_UART_Receive_IT(&huart2, uart2RxData, QUERY_SIZE);
	if (res != HAL_OK)
		Error_Handler();

	nextionObjectInit();
	initArterialIOs();

//	configureTimer2();
	configureTimer3();
//	startTimer2();
	startTimer3();

	//HAL_IWDG_Refresh(&hiwdg);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		uartReinit(5000);
		processRs485Cmd();

		enableKeepAliveLed();

		updateAlarm();

		updateArterialValues();
		updateDlValues();
		update_dc_voltage_values();

		if (((HAL_GetTick() - sendDataToNextionTicks)
				> SEND_DATA_TO_NEXION_TIMEOUT)) {
			sendDataToNextionTicks = HAL_GetTick();
			send_data_to_nextion_on_timeout();
		}

		HAL_IWDG_Refresh(&hiwdg);

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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
	hi2c2.Init.Timing = 0x20303E5D;
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
