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
#define ARTERIAL1_48V_OBJECT_NAME "t4\0"
#define ARTERIAL1_24V_OBJECT_NAME "t5\0"
#define ARTERIAL1_12V_OBJECT_NAME "t6\0"
#define ARTERIAL1_CURRENT_OBJECT_NAME "x1\0"
#define ARTERIAL1_CURRENT_BAR_NAME "j0\0"
#define ARTERIAL1_POWER_OBJECT_NAME "x2\0"
#define ARTERIAL1_DC_ON_OBJECT_NAME "bt1\0"
#define ARTERIAL1_RF_ON_OBJECT_NAME "bt0\0"
#define ARTERIAL1_DC_ON_OBJECT_ID 5
#define ARTERIAL1_RF_ON_OBJECT_ID 3
#define ARTERIAL1_CURRENT_OBJECT_ID 4

#define ARTERIAL2_VOLTAGE_OBJECT_NAME "x5\0"
#define ARTERIAL2_48V_OBJECT_NAME "t7\0"
#define ARTERIAL2_24V_OBJECT_NAME "t8\0"
#define ARTERIAL2_12V_OBJECT_NAME "t9\0"
#define ARTERIAL2_CURRENT_OBJECT_NAME "x4\0"
#define ARTERIAL2_CURRENT_BAR_NAME "j1\0"
#define ARTERIAL2_POWER_OBJECT_NAME "x3\0"
#define ARTERIAL2_DC_ON_OBJECT_NAME "bt3\0"
#define ARTERIAL2_RF_ON_OBJECT_NAME "bt2\0"
#define ARTERIAL2_DC_ON_OBJECT_ID 13
#define ARTERIAL2_RF_ON_OBJECT_ID 12
#define ARTERIAL2_CURRENT_OBJECT_ID 6

#define ARTERIAL3_VOLTAGE_OBJECT_NAME "x8\0"
#define ARTERIAL3_48V_OBJECT_NAME "t10\0"
#define ARTERIAL3_24V_OBJECT_NAME "t11\0"
#define ARTERIAL3_12V_OBJECT_NAME "t12\0"
#define ARTERIAL3_CURRENT_OBJECT_NAME "x7\0"
#define ARTERIAL3_CURRENT_BAR_NAME "j2\0"
#define ARTERIAL3_POWER_OBJECT_NAME "x6\0"
#define ARTERIAL3_DC_ON_OBJECT_NAME "bt5\0"
#define ARTERIAL3_RF_ON_OBJECT_NAME "bt4\0"
#define ARTERIAL3_DC_ON_OBJECT_ID 15
#define ARTERIAL3_RF_ON_OBJECT_ID 14
#define ARTERIAL3_CURRENT_OBJECT_ID 7

#define ARTERIAL4_VOLTAGE_OBJECT_NAME "x11\0"
#define ARTERIAL4_48V_OBJECT_NAME "t13\0"
#define ARTERIAL4_24V_OBJECT_NAME "t14\0"
#define ARTERIAL4_12V_OBJECT_NAME "t15\0"
#define ARTERIAL4_CURRENT_OBJECT_NAME "x10\0"
#define ARTERIAL4_CURRENT_BAR_NAME "j3\0"
#define ARTERIAL4_POWER_OBJECT_NAME "x9\0"
#define ARTERIAL4_DC_ON_OBJECT_NAME "bt7\0"
#define ARTERIAL4_RF_ON_OBJECT_NAME "bt6\0"
#define ARTERIAL4_DC_ON_OBJECT_ID 17
#define ARTERIAL4_RF_ON_OBJECT_ID 16
#define ARTERIAL4_CURRENT_OBJECT_ID 8

#define UPLINK_OBJECT_NAME "x13\0"
#define DOWNLINK_OBJECT_NAME "x12\0"

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
#define UART2_RX_BUFFLEN 30

#define ADC_MIN_VALUE 0
#define ADC_MAX_VALUE 4095
#define VOLTAGE_MIN 0
#define VOLTAGE_MAX 48
#define DOWNLINK_LEVEL_MIN -45

#define DOWNLINK_LEVEL_MAX 5
#define UPLINK_LEVEL_MIN -45
#define UPLINK_LEVEL_MAX 0

#define ART1_THRESHOLD 1000
#define ART2_THRESHOLD 3212
#define ART3_THRESHOLD 3212
#define ART4_THRESHOLD 3212
#define VOLT_THRESHOLD 1232
#define DOWNLINK_THRESHOLD 1823
#define UPLINK_THRESHOLD 100

#define VOLTAGE_THRESHOLD_5V 500
#define VOLTAGE_THRESHOLD_12V 1200
#define VOLTAGE_THRESHOLD_23V 2000
#define VOLTAGE_THRESHOLD_36V 3000

#define RED_BACKGROUND 63488
#define WHITE_BACKGROUND 65535
#define GREEN_BACKGROUND 1024
#define GREEN_DARK_BACKGROUND 480
#define GREY_BACKGROUND 50712

int uplinkLevel;
int downlinkLevel;
bool voltageAlarm = false;
bool downlinkAlarm = false;
bool uplinkAlarm = false;
GPIO_PinState pinVoltageAlarm;
GPIO_PinState pinDownlinkAlarm;

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

typedef enum {
	VIN_12V, VIN_24V, VIN_48V, VIN_UNKNOW
} VOLTAGE_STATE_T;

typedef struct ARTERIAL {
	float voltage;
	float current;
	VOLTAGE_STATE_T voltageState; // 0  = 12[v] , 1 = 24[
	uint8_t currentPercent;
	uint8_t power;
	char vObjName[4];
	char v12StateObjsName[4];
	char v24StateObjsName[4];
	char v48StateObjsName[4];
	char cObjName[4];
	char cBarObjName[4];
	char pObjName[4];
	char rfObjName[4];
	char dcObjName[4];
	uint8_t vObjID;
	uint8_t cObjID;
	uint8_t pObjID;
	uint8_t rfObjID;
	uint8_t dcObID;
	bool adcCurAlarm;
	GPIO_PinState pinCurAlarm;
} Arterial;

// Create a lookup table to map arterial numbers to ADC indices
const int arterialToADCIndex[ARTERIAL_NUMBER] = { ART1_CURRENT_CH,
		ART2_CURRENT_CH, ART3_CURRENT_CH, ART4_CURRENT_CH };

typedef struct {
	GPIO_TypeDef *dcPort;
	uint16_t dcPin;
	GPIO_TypeDef *rfPort;
	uint16_t rfPin;
	bool dcOn;
	bool rfOn;
} ArterialIO;

Arterial arterial[ARTERIAL_NUMBER];
ArterialIO arterialIOs[ARTERIAL_NUMBER] = { { DC_A1_GPIO_Port,
DC_A1_Pin, RF_A1_GPIO_Port, RF_A1_Pin, false,
false }, { DC_A2_GPIO_Port, DC_A2_Pin,
RF_A2_GPIO_Port, RF_A2_Pin, false, false }, {
DC_A3_GPIO_Port, DC_A3_Pin, RF_A3_GPIO_Port,
RF_A3_Pin, false, false }, { DC_A4_GPIO_Port,
DC_A4_Pin, RF_A4_GPIO_Port, RF_A4_Pin, false,
false } };

// UART1 variables for nextion comunication
uint8_t uart1RxData[BUTTON_DATA_SIZE];
uint8_t Cmd_End[3] = { 0xff, 0xff, 0xff };
bool sendData = false;

// UART2 variables for parameter quering
typedef enum MODULE_FUNCTION {
	SERVER,
	QUAD_BAND,
	PSU,
	TETRA,
	ULADR,
	VLADR,
	BDA,
	LOW_NOISE_AMPLIFIER,
	POWER_AMPLIFIER,
	UHF_TONE,
	HEAD_END
} Function_t;

typedef enum RS485_CMD {
	NONE,
	QUERY_MODULE_ID = 0x10,
	QUERY_STATUS,
	SET_VLAD_ATTENUATION,
	QUERY_MASTER_STATUS,

	QUERY_TX_FREQ = 0x20,
	QUERY_RX_FREQ,
	QUERY_UART_BAUDRATE,
	QUERY_BANDWIDTH,
	QUERY_SPREAD_FACTOR,
	QUERY_CODING_RATE,

	SET_MODULE_ID = 0x90,
	SET_TX_FREQ = 0xB0,
	SET_RX_FREQ,
	SET_UART_BAUDRATE,
	SET_BANDWIDTH,
	SET_SPREAD_FACTOR,
	SET_CODING_RATE,

	SET_VLAD_MODE,
	SET_PARAMETER_FREQOUT = 0x31,
	SET_PARAMETERS,
	SET_PARAMETER_FREQBASE,
	QUERY_PARAMETER_PdBm,
} Rs485_cmd_t;

uint8_t rxData;
uint8_t uart2RxSize = 0;
uint8_t uart2RxData[UART2_RX_BUFFLEN];
bool isRxDataReady = false;

uint8_t txData;
uint8_t uart2TxIndex = 0;
uint8_t uart2TxData[UART2_RX_BUFFLEN];
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

void sendColorToNextion(char *obj, uint32_t color) {
	// convert to the integer
	uint8_t len = 0;
	uint8_t buffer[30] = { 0 };
	len = sprintf((char*) buffer, "%s.bco=%lu", obj, color);
	HAL_UART_Transmit(&huart1, buffer, len, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, Cmd_End, sizeof(Cmd_End), HAL_MAX_DELAY);
}

void sendArterialToNextion(Arterial arterial) {
	sendFloatToNextion(arterial.vObjName, arterial.voltage, 2);
	sendFloatToNextion(arterial.cObjName, arterial.current, 2);
	sendFloatToNextion(arterial.pObjName, arterial.power, 2);
	if (arterial.adcCurAlarm)
		sendColorToNextion(arterial.cObjName, RED_BACKGROUND);
	else
		sendColorToNextion(arterial.cObjName, WHITE_BACKGROUND);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart == &huart1) {
		if ((uart1RxData[START] == *START_RX_DATA)
				&& (uart1RxData[END] == *END_RX_DATA)) {
			for (uint8_t i = 0; i < ARTERIAL_NUMBER; i++) {
				if (uart1RxData[CMD] == arterial[i].dcObID)
					arterialIOs[i].dcOn = uart1RxData[VALUE];

				else if (uart1RxData[CMD] == arterial[i].rfObjID)
					arterialIOs[i].rfOn = uart1RxData[VALUE];
			}
			sendData = true;
		}
		huart->RxXferCount = 0;
		HAL_UART_Receive_IT(huart, uart1RxData, BUTTON_DATA_SIZE);
	}
	if (huart == &huart2) {
		/* Read received data from UART1 */
		if (uart2RxSize >= UART2_RX_BUFFLEN) {
			memset(uart2RxData, 0, UART2_RX_BUFFLEN);
			uart2RxSize = 0;
		}
		HAL_UART_Receive_IT(huart, &rxData, 1);
		uart2RxData[uart2RxSize++] = rxData;
		if (rxData == RDSS_END_MARK)
			isRxDataReady = true;
	}
}

uint16_t crc_get(uint8_t *buffer, uint8_t buff_len) {
	uint8_t byte_idx;
	uint8_t bit_idx;
	uint16_t generator = 0x1021; // 16-bit divisor
	uint16_t crc = 0;            // 16-bit CRC value

	for (byte_idx = 0; byte_idx < buff_len; byte_idx++) {
		crc ^= ((uint16_t) (buffer[byte_idx] << 8)); // Move byte into MSB of 16-bit CRC

		for (bit_idx = 0; bit_idx < 8; bit_idx++) {
			if ((crc & 0x8000) != 0) { // Test for MSB = bit 15
				crc = ((uint16_t) ((crc << 1) ^ generator));
			} else {
				crc <<= 1;
			}
		}
	}

	return (crc);
}

uint8_t setCrc(uint8_t *buff, uint8_t size) {
	uint8_t crc_frame[2];
	uint16_t crc;
	crc = crc_get(buff + 1, size - 1);
	memcpy(crc_frame, &crc, 2);
	buff[size++] = crc_frame[0];
	buff[size++] = crc_frame[1];
	return (2);
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

	const char *voltage12ObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_12V_OBJECT_NAME, ARTERIAL2_12V_OBJECT_NAME,
	ARTERIAL3_12V_OBJECT_NAME, ARTERIAL4_12V_OBJECT_NAME };

	const char *voltage24ObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_24V_OBJECT_NAME, ARTERIAL2_24V_OBJECT_NAME,
	ARTERIAL3_24V_OBJECT_NAME, ARTERIAL4_24V_OBJECT_NAME };

	const char *voltage48ObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_48V_OBJECT_NAME, ARTERIAL2_48V_OBJECT_NAME,
	ARTERIAL3_48V_OBJECT_NAME, ARTERIAL4_48V_OBJECT_NAME };

	const char *currentObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_CURRENT_OBJECT_NAME, ARTERIAL2_CURRENT_OBJECT_NAME,
	ARTERIAL3_CURRENT_OBJECT_NAME, ARTERIAL4_CURRENT_OBJECT_NAME };
	const char *currentBarObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_CURRENT_BAR_NAME, ARTERIAL2_CURRENT_BAR_NAME,
	ARTERIAL3_CURRENT_BAR_NAME, ARTERIAL4_CURRENT_BAR_NAME };
	const char *powerObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_POWER_OBJECT_NAME, ARTERIAL2_POWER_OBJECT_NAME,
	ARTERIAL3_POWER_OBJECT_NAME, ARTERIAL4_POWER_OBJECT_NAME };
	const char *dcOnObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_DC_ON_OBJECT_NAME, ARTERIAL2_DC_ON_OBJECT_NAME,
	ARTERIAL3_DC_ON_OBJECT_NAME, ARTERIAL4_DC_ON_OBJECT_NAME };
	const char *rfOnObjectNames[ARTERIAL_NUMBER] = {
	ARTERIAL1_RF_ON_OBJECT_NAME, ARTERIAL2_RF_ON_OBJECT_NAME,
	ARTERIAL3_RF_ON_OBJECT_NAME, ARTERIAL4_RF_ON_OBJECT_NAME };
	const int currentObjectIDs[ARTERIAL_NUMBER] = { ARTERIAL1_CURRENT_OBJECT_ID,
	ARTERIAL2_CURRENT_OBJECT_ID, ARTERIAL3_CURRENT_OBJECT_ID,
	ARTERIAL4_CURRENT_OBJECT_ID };

	const int dcObjectIDs[ARTERIAL_NUMBER] = { ARTERIAL1_DC_ON_OBJECT_ID,
	ARTERIAL2_DC_ON_OBJECT_ID, ARTERIAL3_DC_ON_OBJECT_ID,
	ARTERIAL4_DC_ON_OBJECT_ID };
	const int rfObjectIDs[ARTERIAL_NUMBER] = { ARTERIAL1_RF_ON_OBJECT_ID,
	ARTERIAL2_RF_ON_OBJECT_ID, ARTERIAL3_RF_ON_OBJECT_ID,
	ARTERIAL4_RF_ON_OBJECT_ID };
	for (uint8_t i = 0; i < ARTERIAL_NUMBER; i++) {
		sprintf(arterial[i].vObjName, voltageObjectNames[i]);
		sprintf(arterial[i].v12StateObjsName, voltage12ObjectNames[i]);
		sprintf(arterial[i].v24StateObjsName, voltage24ObjectNames[i]);
		sprintf(arterial[i].v48StateObjsName, voltage48ObjectNames[i]);
		sprintf(arterial[i].cObjName, currentObjectNames[i]);
		sprintf(arterial[i].cBarObjName, currentBarObjectNames[i]);
		sprintf(arterial[i].pObjName, powerObjectNames[i]);
		sprintf(arterial[i].dcObjName, dcOnObjectNames[i]);
		sprintf(arterial[i].rfObjName, rfOnObjectNames[i]);

		arterial[i].cObjID = currentObjectIDs[i];
		arterial[i].dcObID = dcObjectIDs[i];
		arterial[i].rfObjID = rfObjectIDs[i];
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

float arduino_map(uint16_t value, uint16_t in_min, uint16_t in_max,
		float out_min, float out_max) {
	return ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void updateArterialData() {
	for (int numr = 0; numr < ARTERIAL_NUMBER; numr++) {
		HAL_GPIO_WritePin(arterialIOs[numr].dcPort, arterialIOs[numr].dcPin,
				arterialIOs[numr].dcOn);
		HAL_GPIO_WritePin(arterialIOs[numr].rfPort, arterialIOs[numr].rfPin,
				arterialIOs[numr].rfOn);

		arterial[numr].voltage = arduino_map(adcValues[VOLTAGE_CH],
		ADC_MIN_VALUE, ADC_MAX_VALUE, VOLTAGE_MIN, VOLTAGE_MAX);

		if (adcValues[VOLTAGE_CH] >= VOLTAGE_THRESHOLD_5V
				&& adcValues[VOLTAGE_CH] < VOLTAGE_THRESHOLD_12V) {
			arterial[numr].voltageState = VIN_12V;
		} else if (adcValues[VOLTAGE_CH] >= VOLTAGE_THRESHOLD_23V
				&& adcValues[VOLTAGE_CH] < VOLTAGE_THRESHOLD_36V) {
			arterial[numr].voltageState = VIN_24V;
		} else if (adcValues[VOLTAGE_CH] >= VOLTAGE_THRESHOLD_36V) {
			arterial[numr].voltageState = VIN_48V;
		} else {
			arterial[numr].voltageState = VIN_UNKNOW;
		}
		int adcIndex = arterialToADCIndex[numr];
		arterial[numr].current = arduino_map(adcValues[adcIndex],
		ADC_MIN_VALUE,
		ADC_MAX_VALUE, 0, 3);
		arterial[numr].currentPercent = arduino_map(adcValues[adcIndex],
		ADC_MIN_VALUE,
		ADC_MAX_VALUE, 0, 100);

		arterial[numr].power = arterial[numr].voltage * arterial[numr].current;

	}

	// Map downlink and uplink levels
	downlinkLevel = arduino_map(adcValues[DOWNLINK_LVL_CH], ADC_MIN_VALUE,
	ADC_MAX_VALUE, DOWNLINK_LEVEL_MIN, DOWNLINK_LEVEL_MAX);
	uplinkLevel = arduino_map(adcValues[UPLINK_LVL_CH], ADC_MIN_VALUE,
	ADC_MAX_VALUE, UPLINK_LEVEL_MIN, UPLINK_LEVEL_MAX);
}

uint8_t updateUART2Tx(uint8_t *dataSize) {
	uint8_t idx = 0;
	uart2TxData[idx++] = RDSS_START_MARK;
	uart2TxData[idx++] = HEAD_END;
	uart2TxData[idx++] = 0x00;
	uart2TxData[idx++] = QUERY_STATUS;
	uart2TxData[idx++] = 0x00;
	uart2TxData[idx++] = *dataSize;
	*dataSize = idx; // 6
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
	*dataSize = idx - *dataSize; //21 - 6 = 15
	setCrc(uart2TxData, idx);
	uart2TxData[idx + 2] = RDSS_END_MARK;
	return (idx);
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
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */

	// Enable the timer clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	// Configure the timer
	TIM2->PSC = 7999; // Prescaler value for a 1 ms time base (assuming a 48 MHz clock)
	TIM2->ARR = 499;  // Auto-reload value for a 500 ms period
//	TIM2->ARR = 3999;  // Auto-reload value for a 500 ms period
	TIM2->CR1 |= TIM_CR1_ARPE;  // Enable auto-reload preload
	TIM2->CR1 |= TIM_CR1_URS; // Only overflow/underflow generates an update interrupt
	TIM2->DIER |= TIM_DIER_UIE;  // Enable update interrupt

	// Initialize necessary peripherals and configurations
	TIM2_Interrupt_Init();
	enableGlobalInterrupts();

//	res = HAL_ADC_Start_DMA(&hadc, (uint32_t*) adcValues, ADC_CHANNELS);
//	if (res != HAL_OK)
//		Error_Handler();

	res = HAL_UART_Receive_IT(&huart1, uart1RxData, BUTTON_DATA_SIZE);
	if (res != HAL_OK)
		Error_Handler();

	uint32_t nextionSendTicks = HAL_GetTick();
	uint32_t nextionSendTimeout = 1000;
	uint8_t dataSize = 0;
	arterialInit(arterial, arterialIOs);
	startTimer2();
	HAL_IWDG_Refresh(&hiwdg);
	// ...
	/* USER CODE END 2 */
	srand(5000);

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		for (int numr = 0; numr < ARTERIAL_NUMBER; numr++) {
			int adcIndex = arterialToADCIndex[numr];
			if (arterialIOs[numr].dcOn) {
				adcValues[adcIndex] = rand() % 4095;

			} else {
				adcValues[adcIndex] = 0;
			}
		}
		adcValues[VOLTAGE_CH] = rand() % 4095;
		updateArterialData();

		updateUART2Tx(&dataSize);

		if (sendData) {
			sendData = false;
			for (uint8_t adcIdx = 0; adcIdx < ADC_CHANNELS; adcIdx++) {
				if (adcIdx == ART1_CURRENT_CH) {
					arterial[ART1].adcCurAlarm = adcValues[adcIdx]
							> ART1_THRESHOLD;
				} else if (adcIdx == ART2_CURRENT_CH) {
					arterial[ART2].adcCurAlarm = adcValues[adcIdx]
							> ART2_THRESHOLD;
				} else if (adcIdx == ART3_CURRENT_CH) {
					arterial[ART3].adcCurAlarm = adcValues[adcIdx]
							> ART3_THRESHOLD;
				} else if (adcIdx == ART4_CURRENT_CH) {
					arterial[ART4].adcCurAlarm = adcValues[adcIdx]
							> ART4_THRESHOLD;
				} else if (adcIdx == VOLTAGE_CH) {
					voltageAlarm = adcValues[adcIdx] > VOLT_THRESHOLD;
				} else if (adcIdx == DOWNLINK_LVL_CH) {
					downlinkAlarm = adcValues[adcIdx] > DOWNLINK_THRESHOLD;
				} else if (adcIdx == UPLINK_LVL_CH) {
					uplinkAlarm = adcValues[adcIdx] > UPLINK_THRESHOLD;
				} else {
					// Handle other cases if necessary
				}
			}

			for (int i = 0; i < ARTERIAL_NUMBER; i++) {
				//sendFloatToNextion(arterial[i].vObjName, arterial[i].voltage,
				sendFloatToNextion(arterial[i].cObjName, arterial[i].current,
						2);
				sendNumToNextion(arterial[i].cBarObjName,
						arterial[i].currentPercent);

				if (arterialIOs[i].dcOn) {
					if (arterial[i].voltageState == VIN_12V) {
						sendColorToNextion(arterial[i].v12StateObjsName,
						GREEN_BACKGROUND);
						sendColorToNextion(arterial[i].v24StateObjsName,
						WHITE_BACKGROUND);
						sendColorToNextion(arterial[i].v48StateObjsName,
						WHITE_BACKGROUND);
					} else if (arterial[i].voltageState == VIN_24V) {
						sendColorToNextion(arterial[i].v12StateObjsName,
						WHITE_BACKGROUND);
						sendColorToNextion(arterial[i].v24StateObjsName,
						GREEN_BACKGROUND);
						sendColorToNextion(arterial[i].v48StateObjsName,
						WHITE_BACKGROUND);
					} else if (arterial[i].voltageState == VIN_48V) {
						sendColorToNextion(arterial[i].v12StateObjsName,
						WHITE_BACKGROUND);
						sendColorToNextion(arterial[i].v24StateObjsName,
						WHITE_BACKGROUND);
						sendColorToNextion(arterial[i].v48StateObjsName,
						GREEN_BACKGROUND);
					} else {
						sendColorToNextion(arterial[i].v12StateObjsName,
						WHITE_BACKGROUND);
						sendColorToNextion(arterial[i].v24StateObjsName,
						WHITE_BACKGROUND);
						sendColorToNextion(arterial[i].v48StateObjsName,
						WHITE_BACKGROUND);
					}
				} else {

					if (arterial[i].voltageState == VIN_12V) {
						sendColorToNextion(arterial[i].v12StateObjsName,
						GREEN_DARK_BACKGROUND);
						sendColorToNextion(arterial[i].v24StateObjsName,
						GREY_BACKGROUND);
						sendColorToNextion(arterial[i].v48StateObjsName,
						GREY_BACKGROUND);
					} else if (arterial[i].voltageState == VIN_24V) {
						sendColorToNextion(arterial[i].v12StateObjsName,
						GREY_BACKGROUND);
						sendColorToNextion(arterial[i].v24StateObjsName,
						GREEN_DARK_BACKGROUND);
						sendColorToNextion(arterial[i].v48StateObjsName,
						GREY_BACKGROUND);
					} else if (arterial[i].voltageState == VIN_48V) {
						sendColorToNextion(arterial[i].v12StateObjsName,
						GREY_BACKGROUND);
						sendColorToNextion(arterial[i].v24StateObjsName,
						GREY_BACKGROUND);
						sendColorToNextion(arterial[i].v48StateObjsName,
						GREEN_DARK_BACKGROUND);
					} else {
						sendColorToNextion(arterial[i].v12StateObjsName,
						GREY_BACKGROUND);
						sendColorToNextion(arterial[i].v24StateObjsName,
						GREY_BACKGROUND);
						sendColorToNextion(arterial[i].v48StateObjsName,
						GREY_BACKGROUND);
					}
				}
				if (arterial[i].adcCurAlarm)
					sendColorToNextion(arterial[i].cObjName,
					RED_BACKGROUND);
				else
					sendColorToNextion(arterial[i].cObjName,
					WHITE_BACKGROUND);
			}
			sendFloatToNextion(DOWNLINK_OBJECT_NAME, downlinkLevel, 2);
//			sendFloatToNextion(UPLINK_OBJECT_NAME, uplinkLevel, 2);
		}

		arterial[ART1].pinCurAlarm = HAL_GPIO_ReadPin(ALARM_A1_GPIO_Port,
		ALARM_A1_Pin);	//ESTADO ALARM1 art4 > 3
		arterial[ART2].pinCurAlarm = HAL_GPIO_ReadPin(ALARM_A2_GPIO_Port,
		ALARM_A2_Pin);	//ESTADO ALARM2 art3 >
		arterial[ART3].pinCurAlarm = HAL_GPIO_ReadPin(ALARM_A3_GPIO_Port,
		ALARM_A3_Pin);	//ESTADO ALARM3 art2 >
		arterial[ART4].pinCurAlarm = HAL_GPIO_ReadPin(ALARM_A4_GPIO_Port,
		ALARM_A4_Pin);	//ESTADO ALARM4 art1 >
		pinDownlinkAlarm = HAL_GPIO_ReadPin(ALARM_VIN_GPIO_Port,
		ALARM_VIN_Pin);	//ESTADO ALARM5 fuente >
		HAL_IWDG_Refresh(&hiwdg);
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
	HAL_GPIO_WritePin(GPIOA, DC_A1_Pin | DC_A2_Pin | DC_A3_Pin | DC_A4_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			RF_A4_Pin | RF_A3_Pin | RF_A2_Pin | GPIO_PIN_12 | GPIO_PIN_13
					| GPIO_PIN_14 | GPIO_PIN_15 | RF_A1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : DC_A1_Pin DC_A2_Pin DC_A3_Pin DC_A4_Pin */
	GPIO_InitStruct.Pin = DC_A1_Pin | DC_A2_Pin | DC_A3_Pin | DC_A4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : RF_A4_Pin RF_A3_Pin RF_A2_Pin PB12
	 PB13 PB14 PB15 RF_A1_Pin */
	GPIO_InitStruct.Pin = RF_A4_Pin | RF_A3_Pin | RF_A2_Pin | GPIO_PIN_12
			| GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | RF_A1_Pin;
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
