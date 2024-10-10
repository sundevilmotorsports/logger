/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "string.h"

#include <GNSS.h>
#include "adc.h"
#include "ina260.h"
#include "eeprom.h"
#include "logger.h"
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

FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart9;
DMA_HandleTypeDef hdma_uart9_rx;
DMA_HandleTypeDef hdma_uart9_tx;

/* USER CODE BEGIN PV */
FDCAN_FilterTypeDef canFilter;
FATFS fatFS;
FIL file;
char diskPath[4];
uint8_t rtext[_MAX_SS];/* File read buffer */

uint8_t usbBuffer[64];
//GNSS_StateHandle gps;



enum LogChannel {
	TS, TS1, TS2, TS3, // timestamp (ms)
	F_BRAKEPRESSURE, F_BRAKEPRESSURE1, // front brake pressure
	R_BRAKEPRESSURE, R_BRAKEPRESSURE1, // rear brake pressure
	STEERING, STEERING1, // steering pot
	FLSHOCK, FLSHOCK1,
	FRSHOCK, FRSHOCK1,
	RRSHOCK, RRSHOCK1,
	RLSHOCK, RLSHOCK1,
	CURRENT, CURRENT1,
	BATTERY, BATTERY1,
	IMU_X_ACCEL, IMU_X_ACCEL1, IMU_X_ACCEL2, IMU_X_ACCEL3, // mG
	IMU_Y_ACCEL, IMU_Y_ACCEL1, IMU_Y_ACCEL2, IMU_Y_ACCEL3,
	IMU_Z_ACCEL, IMU_Z_ACCEL1, IMU_Z_ACCEL2, IMU_Z_ACCEL3,
	IMU_X_GYRO, IMU_X_GYRO1, IMU_X_GYRO2, IMU_X_GYRO3, // mdps
	IMU_Y_GYRO, IMU_Y_GYRO1, IMU_Y_GYRO2, IMU_Y_GYRO3,
	IMU_Z_GYRO, IMU_Z_GYRO1, IMU_Z_GYRO2, IMU_Z_GYRO3,
	FR_SG, FR_SG1, // 16 bit adc
	FL_SG, FL_SG1,
	RL_SG, RL_SG1,
	RR_SG, RR_SG1,
	FLW_AMB, FLW_AMB1,
	FLW_OBJ, FLW_OBJ1,
	FLW_RPM, FLW_RPM1,
	FRW_AMB, FRW_AMB1,
	FRW_OBJ, FRW_OBJ1,
	FRW_RPM, FRW_RPM1,
	RRW_AMB, RRW_AMB1,
	RRW_OBJ, RRW_OBJ1,
	RRW_RPM, RRW_RPM1,
	RLW_AMB, RLW_AMB1,
	RLW_OBJ, RLW_OBJ1,
	RLW_RPM, RLW_RPM1,
	BRAKE_FLUID, BRAKE_FLUID1,
	THROTTLE_LOAD, THROTTLE_LOAD1,
	BRAKE_LOAD, BRAKE_LOAD1,
	DRS,
	GPS_LON, GPS_LON1, GPS_LON2, GPS_LON3,
	GPS_LAT, GPS_LAT1, GPS_LAT2, GPS_LAT3,
	GPS_SPD, GPS_SPD1, GPS_SPD2, GPS_SPD3,
	GPS_FIX,
	TESTNO,
	DTC_FLW, DTC_FRW,
	DTC_RLW, DTC_RRW,
	DTC_FBP, DTC_RBP,
	DTC_STP, DTC_FLS,
	DTC_FRS, DTC_RLS,
	DTC_RRS,
	CH_COUNT
};

uint8_t logBuffer[CH_COUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI4_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_UART9_Init(void);
static void MX_I2C2_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_FDCAN2_Init(void);
/* USER CODE BEGIN PFP */
// calculate temperature in celsius
float mlx90614(uint16_t temp);
float mlx90614(uint16_t temp) {
	return (((float) temp * 0.02) - 273.15);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
	uint16_t ambTemp;
	uint16_t objTemp;
	uint16_t rpm;
} wheel_data_s_t;


//DTC Codes
typedef struct {
    uint32_t dtcCodes[1]; // Single 32-bit integer to store 25 bits
} dtc_code_handler;

volatile dtc_code_handler DTC_Error_State_instance = {0};
volatile dtc_code_handler* DTC_Error_State = &DTC_Error_State_instance;

// DTC Bitwise Macros for Updating the Code Status
#define SET_DTC(container, index)   ((container)->dtcCodes[(index) / 32] |= (1U << ((index) % 32)))
#define CLEAR_DTC(container, index) ((container)->dtcCodes[(index) / 32] &= ~(1U << ((index) % 32)))
#define CHECK_DTC(container, index) ((container)->dtcCodes[(index) / 32] & (1U << ((index) % 32)))

// Function to return the DTC error state
volatile uint32_t* DTC_Error_Check() {
    return DTC_Error_State->dtcCodes;
}

//Author of this awful struct: Alex Rumer, the first year (Who let him touch the Datalogger code? ¯\_(ツ)_/¯ )
#pragma pack(push, 1) //Pushes storage boundary to single bit for best storage efficiency
typedef struct {
	uint8_t init:1; //Limits Storage to a single bit address
	uint8_t errState:1; //Limits Storage to a single bit address
	uint8_t DTC_Code_Index:5; //Limits Storage to 5 bit addresses maximizing the value at 32
	//Corresponds to the 32 addresses in the DTC Code Handler

	uint8_t measures; // Goal Number of Measurements to Calculate Average Response Time (MAX: 256)
	uint8_t bufferIndex; //Actual Memory Allocation for number of Messages Received from CAN ID

	uint32_t totalTime; //Time (ms) from Last Average Response Time Calculation
	//This data can be received from the CAN_RDTxR register (I copied the data type hehe)
	uint32_t prevTime;

	uint32_t avgResponse; //Store the calculated average response time
	uint8_t percentOver; //Store the percentage of avg response time over allowed before throwing an error
	uint32_t *timeBuffer;


}can_dtc; //This name needs work I know...
#pragma pack(pop) // Restores default STM32 Byte Padding

//Initialize CAN Device DTC values, defines where to store DTC code and how many times to measure the avg response time
void CAN_DTC_Init(can_dtc *data, uint8_t index, uint8_t measures, uint8_t percentage_over_allowed, uint8_t start_time){
	data->init = 0;
	data->bufferIndex = 0;
	data->errState = 0;
	data->measures = measures;
	data->totalTime = start_time;
	data->prevTime = start_time;
	data->percentOver = percentage_over_allowed;
	data->DTC_Code_Index = index&0x1F; //Bitwise Operation Ensures Index never exceeds 32 and limits Memory Useage

	//Circular Buffer to Ensure a Rolling Average
	data->timeBuffer = (uint32_t *)malloc(measures * sizeof(uint32_t));
    for (int i = 0; i < measures; i++) {
        data->timeBuffer[i] = 0;
    }
	return;
}

//Update the measurement state of the CAN DTC handler
void CAN_DTC_State_Update(can_dtc *data, uint16_t msgTime){
    uint32_t interval = msgTime - data->prevTime;
    data->prevTime = msgTime;

    // Update the total time by subtracting the oldest value and adding the new interval
    data->totalTime = data->totalTime - data->timeBuffer[data->bufferIndex] + interval;

    // Store the new interval in the buffer
    data->timeBuffer[data->bufferIndex] = interval;

    // Update the buffer index
    data->bufferIndex = (data->bufferIndex + 1) % data->measures;
	return;
}

//Update the Average Response Measurement of the CAN DTC handler
void CAN_DTC_Response_Update(can_dtc *data){
	//Error and Value Checking
	if(data->init && !data->errState){
		//Calculate Average Response Time
		data->avgResponse = data->totalTime/data->measures;
	}
	return;
}

//Check if the CAN device has not responded lately, if not send an Error code to the DTC
void CAN_DTC_Error_Update(can_dtc *data, uint32_t time){
	uint32_t currentTime = time - data->prevTime;
	CAN_DTC_Response_Update(data);

	if(currentTime > data->avgResponse * (1 + data->percentOver/100)){
		SET_DTC(DTC_Error_State, data->DTC_Code_Index);
	}
	return;
}

//Simplified checking logic for switch case checks
void CAN_DTC_Update_All(can_dtc *data, uint32_t time) {
    if (data->init) {
        CAN_DTC_State_Update(data, time);
        if (data->bufferIndex == 0) { // Update the average when the buffer is full
            CAN_DTC_Response_Update(data);
        }
    } else {
        CAN_DTC_State_Update(data, time);
        data->avgResponse = data->totalTime; // Calculate initial average after first message
        data->init = 1; // Set the init flag
    }
}

//Initialize Wheel Board DTC handlers and allocate memory
can_dtc frwDTC_instance, flwDTC_instance, rrwDTC_instance, rlwDTC_instance;
can_dtc *frwDTC = &frwDTC_instance;
can_dtc *flwDTC = &flwDTC_instance;
can_dtc *rrwDTC = &rrwDTC_instance;
can_dtc *rlwDTC = &rlwDTC_instance;


//Defining the DTC Storage Index for each recorded DTC device
uint8_t DTC_Index_frWheelBoard = 0;
uint8_t DTC_Index_flWheelBoard = 1;
uint8_t DTC_Index_rrWheelBoard = 2;
uint8_t DTC_Index_rlWheelBoard = 3;
uint8_t DTC_Index_fBrakePress = 4;
uint8_t DTC_Index_rBrakePress = 5;
uint8_t DTC_Index_steer = 6;
uint8_t DTC_Index_flShock = 7;
uint8_t DTC_Index_frShock = 8;
uint8_t DTC_Index_rlShock = 9;
uint8_t DTC_Index_rrShock = 10;

void DTC_Init(uint32_t start_time){
	CAN_DTC_Init(frwDTC, DTC_Index_frWheelBoard, 10, 25, start_time);
	CAN_DTC_Init(flwDTC, DTC_Index_flWheelBoard, 10, 25, start_time);
	CAN_DTC_Init(rrwDTC, DTC_Index_rrWheelBoard, 10, 25, start_time);
	CAN_DTC_Init(rlwDTC, DTC_Index_rlWheelBoard, 10, 25, start_time);
	for(int i=0; i<32; i++)CLEAR_DTC(DTC_Error_State, i);
	return;
}
void DTC_Error_All(uint32_t time){
	CAN_DTC_Error_Update(frwDTC, time);
	CAN_DTC_Error_Update(flwDTC, time);
	CAN_DTC_Error_Update(rrwDTC, time);
	CAN_DTC_Error_Update(rlwDTC, time);
	return;

}

const uint32_t DTC_CHECK_INTERVAL = 20;
uint32_t DTC_PREV_CHECK_TIME = 0;

FDCAN_RxHeaderTypeDef	RxHeader;
uint8_t               	RxData[8];
volatile uint32_t count = 0;
volatile uint32_t xAccel = 0, yAccel = 0, zAccel = 0;
volatile uint32_t xGyro = 0, yGyro = 0, zGyro = 0;
volatile uint16_t frsg = 0, flsg = 0, rrsg = 0, rlsg = 0;
volatile wheel_data_s_t frw, flw, rlw, rrw;
volatile uint8_t testNo = 0;
volatile uint8_t canFifoFull = 0;
volatile uint8_t drs = 0;
volatile uint16_t brakeFluid = 0, throttleLoad = 0, brakeLoad = 0;

ADC_Result fBrakePress;
ADC_Result rBrakePress;
ADC_Result steer;
ADC_Result flShock;
ADC_Result frShock;
ADC_Result rrShock;
ADC_Result rlShock;




void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
    /* Reception Error */
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    	return;
    	//Error_Handler();
    }
    // do things with data
    canFifoFull = 0;
    count += 1;

    if (count > 900000) {
    	count = 0;
    }
    switch(RxHeader.Identifier) {
    case 0x35F:
    	drs = RxData[0];
    	break;
    case 0x360:
    	xAccel = RxData[0] << 24 | RxData[1] << 16 | RxData[2] << 8 | RxData[3];
    	yAccel = RxData[4] << 24 | RxData[5] << 16 | RxData[6] << 8 | RxData[7];
    	break;
    case 0x361:
    	zAccel = RxData[0] << 24 | RxData[1] << 16 | RxData[2] << 8 | RxData[3];
    	xGyro = RxData[4] << 24 | RxData[5] << 16 | RxData[6] << 8 | RxData[7];
    	break;
    case 0x362:
    	yGyro = RxData[0] << 24 | RxData[1] << 16 | RxData[2] << 8 | RxData[3];
    	zGyro = RxData[4] << 24 | RxData[5] << 16 | RxData[6] << 8 | RxData[7];
    	break;
    case 0x363:
    	//Front Left Wheel Board
    	flw.rpm = RxData[0] << 8 | RxData[1];
    	flw.objTemp = RxData[2] << 8 | RxData[3];
    	flw.ambTemp = RxData[4] << 8 | RxData[5];

    	//If statement ensures update runs only at 50Hz
    	if(HAL_GetTick() - flwDTC->prevTime >= DTC_CHECK_INTERVAL) CAN_DTC_Update_All(flwDTC, HAL_GetTick());

    	break;
    case 0x364:
    	//Front Right Wheel Board
    	frw.rpm = RxData[0] << 8 | RxData[1];
    	frw.objTemp = RxData[2] << 8 | RxData[3];
    	frw.ambTemp = RxData[4] << 8 | RxData[5];

    	//If statement ensures update runs only at 50Hz
    	if(HAL_GetTick() - frwDTC->prevTime >= DTC_CHECK_INTERVAL) CAN_DTC_Update_All(frwDTC, HAL_GetTick());

    	break;
    case 0x365:
    	//Rear Right Wheel Board
    	rrw.rpm = RxData[0] << 8 | RxData[1];
    	rrw.objTemp = RxData[2] << 8 | RxData[3];
    	rrw.ambTemp = RxData[4] << 8 | RxData[5];

    	//If statement ensures update runs only at 50Hz
    	if(HAL_GetTick() - rrwDTC->prevTime >= DTC_CHECK_INTERVAL) CAN_DTC_Update_All(rrwDTC, HAL_GetTick());

    	break;
    case 0x366:
    	//Rear Left Wheel Board
    	rlw.rpm = RxData[0] << 8 | RxData[1];
    	rlw.objTemp = RxData[2] << 8 | RxData[3];
    	rlw.ambTemp = RxData[4] << 8 | RxData[5];

    	//If statement ensures update runs only at 50Hz
    	if(HAL_GetTick() - rlwDTC->prevTime >= DTC_CHECK_INTERVAL) CAN_DTC_Update_All(rlwDTC, HAL_GetTick());

    	break;
    case 0x368:
    	//Brake Fluid, Throttle Load, and Brake Load
    	brakeFluid = RxData[0] << 8 | RxData[1];
    	throttleLoad = RxData[2] << 8 | RxData[3];
    	brakeLoad = RxData[4] << 8 | RxData[5];
    	break;
    case 0x4e2:
    	//Front Left String Gauge
    	flsg = RxData[0] << 8 | RxData[1];
    	break;
    case 0x4e3:
    	//Front Right String Gauge
    	frsg = RxData[0] << 8 | RxData[1];
    	break;
    case 0x4e4:
    	//Rear Right String Gauge
    	rrsg = RxData[0] << 8 | RxData[1];
    	break;
    case 0x4e5:
    	//Rear Left String Gauge
    	rlsg = RxData[0] << 8 | RxData[1];
    	break;
    }


    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      //Error_Handler();
    }
  }
  else if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != RESET) {
	  canFifoFull = 1;
	  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_FULL, 0);
  }
}

int _write( int file, char *ptr, int len )
{
	int i = 0;
	for( i=0 ; i < len ; i++ ) ITM_SendChar(( *ptr++));
	return len;
}

void SWD_Init(void)
{
	*(__IO uint32_t*) (0x5C003010) = ((SystemCoreClock / 2 / 2000000) - 1);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  SWD_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C4_Init();
  MX_SPI4_Init();
  MX_USB_DEVICE_Init();
  MX_SDMMC1_SD_Init();
  MX_UART9_Init();
  MX_FATFS_Init();
  MX_I2C2_Init();
  MX_FDCAN3_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */
  BSP_SD_Init();

  //Initialize DTC subsystem
  DTC_Init(HAL_GetTick());


  GNSS_Init(&GNSS_Handle, &huart9);
  GNSS_GetUniqID(&GNSS_Handle);
  HAL_Delay(1000);
  GNSS_ParseBuffer(&GNSS_Handle);
  //GNSS_LoadConfig(&GNSS_Handle); // DO NOT LOAD CONFIG IT WILL BREAK GPS
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char msg[128];
  char name[16];

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  UINT wbytes; // unsigned int

  uint8_t runNo = 0;
  uint16_t runNoAddr = 4;
  eepromRead(&hi2c2, runNoAddr, &runNo);
  uint8_t currRunNo = runNo;

  sprintf(name, "data%d.benji", runNo);
  if(++runNo == 255) {
	  runNo = 0;
  }
  eepromWrite(&hi2c2, runNoAddr, &runNo);



  if(f_mount(&fatFS, (TCHAR const*) diskPath, 0) == FR_OK)
  {
	  FRESULT res = FR_OK; //f_mkfs((TCHAR const*) diskPath, FM_ANY, 0, rtext, sizeof(rtext));
	  if(res != FR_OK)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		  Error_Handler();
	  }
  }

  if(f_open(&file, (TCHAR const*) name, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  Error_Handler();
  }



  while (1)
  {
	  //Check the Error State of all the DTC devices at 50 Hz
	  if(HAL_GetTick() - DTC_PREV_CHECK_TIME >= DTC_CHECK_INTERVAL ){
		  //Check CAN Device Response
		  DTC_Error_All(HAL_GetTick());

		  //Check Analog Device Response
		  if(fBrakePress.error!=HAL_OK) SET_DTC(DTC_Error_State, DTC_Index_fBrakePress);
		  if(rBrakePress.error!=HAL_OK) SET_DTC(DTC_Error_State, DTC_Index_rBrakePress);
		  if(steer.error!=HAL_OK) SET_DTC(DTC_Error_State, DTC_Index_steer);
		  if(flShock.error!=HAL_OK) SET_DTC(DTC_Error_State, DTC_Index_flShock);
		  if(frShock.error!=HAL_OK) SET_DTC(DTC_Error_State, DTC_Index_frShock);
		  if(rlShock.error!=HAL_OK) SET_DTC(DTC_Error_State, DTC_Index_rlShock);
		  if(rrShock.error!=HAL_OK) SET_DTC(DTC_Error_State, DTC_Index_rrShock);

		  //Update Last Check Time
		  DTC_PREV_CHECK_TIME = HAL_GetTick();
	  }

	  //Start Analog Listening
	  adcEnable();

	  //Receive Analog Sensor Data and Store
	  fBrakePress = eGetAnalog(&hspi4, ADC_FBP);
	  rBrakePress = eGetAnalog(&hspi4, ADC_RBP);
	  steer = eGetAnalog(&hspi4, ADC_STP);
	  flShock = eGetAnalog(&hspi4, ADC_FLS);
	  frShock = eGetAnalog(&hspi4, ADC_FRS);
	  rrShock = eGetAnalog(&hspi4, ADC_RRS);
	  rlShock = eGetAnalog(&hspi4, ADC_RLS);


	  //Log Analog Sensor Data
	  loggerEmplaceU16(logBuffer, F_BRAKEPRESSURE, fBrakePress.value);
	  loggerEmplaceU16(logBuffer, R_BRAKEPRESSURE, rBrakePress.value);
	  loggerEmplaceU16(logBuffer, STEERING, steer.value);
	  loggerEmplaceU16(logBuffer, FLSHOCK, flShock.value);
	  loggerEmplaceU16(logBuffer, FRSHOCK, frShock.value);
	  loggerEmplaceU16(logBuffer, RRSHOCK, rrShock.value);
	  loggerEmplaceU16(logBuffer, RLSHOCK, rlShock.value);

	  //Stop Analog Listening
	  adcDisable();

	  //Report Battery Current and Voltage
	  loggerEmplaceU16(logBuffer, CURRENT, getCurrent(&hi2c4));
	  loggerEmplaceU16(logBuffer, BATTERY, getVoltage(&hi2c4));

	  //Report IMU Data
	  loggerEmplaceU32(logBuffer, IMU_X_ACCEL, xAccel);
	  loggerEmplaceU32(logBuffer, IMU_Y_ACCEL, yAccel);
	  loggerEmplaceU32(logBuffer, IMU_Z_ACCEL, zAccel);

	  loggerEmplaceU32(logBuffer, IMU_X_GYRO, xGyro);
	  loggerEmplaceU32(logBuffer, IMU_Y_GYRO, yGyro);
	  loggerEmplaceU32(logBuffer, IMU_Z_GYRO, zGyro);

	  //Report Wheel Board Sensor Data
	  loggerEmplaceU16(logBuffer, FLW_AMB, flw.ambTemp);
	  loggerEmplaceU16(logBuffer, FLW_OBJ, flw.objTemp);
	  loggerEmplaceU16(logBuffer, FLW_RPM, flw.rpm);

	  loggerEmplaceU16(logBuffer, FRW_AMB, frw.ambTemp);
	  loggerEmplaceU16(logBuffer, FRW_OBJ, frw.objTemp);
	  loggerEmplaceU16(logBuffer, FRW_RPM, frw.rpm);

	  loggerEmplaceU16(logBuffer, RRW_AMB, rrw.ambTemp);
	  loggerEmplaceU16(logBuffer, RRW_OBJ, rrw.objTemp);
	  loggerEmplaceU16(logBuffer, RRW_RPM, rrw.rpm);

	  loggerEmplaceU16(logBuffer, RLW_AMB, rlw.ambTemp);
	  loggerEmplaceU16(logBuffer, RLW_OBJ, rlw.objTemp);
	  loggerEmplaceU16(logBuffer, RLW_RPM, rlw.rpm);

	  //Report String Gauge Data
	  loggerEmplaceU16(logBuffer, FR_SG, frsg);
	  loggerEmplaceU16(logBuffer, FL_SG, flsg);
	  loggerEmplaceU16(logBuffer, RR_SG, rrsg);
	  loggerEmplaceU16(logBuffer, RL_SG, rlsg);

	  //Report Brakes and Throttle
	  loggerEmplaceU16(logBuffer, BRAKE_FLUID, brakeFluid);
	  loggerEmplaceU16(logBuffer, THROTTLE_LOAD, throttleLoad);
	  loggerEmplaceU16(logBuffer, BRAKE_LOAD, brakeLoad);

	  //Report DTC Data
	  loggerEmplaceU16(logBuffer, DTC_FLW, CHECK_DTC(DTC_Error_State, DTC_Index_flWheelBoard));
	  loggerEmplaceU16(logBuffer, DTC_FRW, CHECK_DTC(DTC_Error_State, DTC_Index_frWheelBoard));
	  loggerEmplaceU16(logBuffer, DTC_RRW, CHECK_DTC(DTC_Error_State, DTC_Index_rrWheelBoard));
	  loggerEmplaceU16(logBuffer, DTC_RLW, CHECK_DTC(DTC_Error_State, DTC_Index_rlWheelBoard));
	  loggerEmplaceU16(logBuffer, DTC_FBP, CHECK_DTC(DTC_Error_State, DTC_Index_fBrakePress));
    loggerEmplaceU16(logBuffer, DTC_RBP, CHECK_DTC(DTC_Error_State, DTC_Index_rBrakePress));
    loggerEmplaceU16(logBuffer, DTC_STP, CHECK_DTC(DTC_Error_State, DTC_Index_steer));
    loggerEmplaceU16(logBuffer, DTC_FLS, CHECK_DTC(DTC_Error_State, DTC_Index_flShock));
    loggerEmplaceU16(logBuffer, DTC_FRS, CHECK_DTC(DTC_Error_State, DTC_Index_frShock));
    loggerEmplaceU16(logBuffer, DTC_RRS, CHECK_DTC(DTC_Error_State, DTC_Index_rrShock));
    loggerEmplaceU16(logBuffer, DTC_RLS, CHECK_DTC(DTC_Error_State, DTC_Index_rlShock));



	  static uint32_t GPS_Timer = 0;
	  if ((HAL_GetTick() - GPS_Timer) > 900) {
		  GNSS_ParseBuffer(&GNSS_Handle);
		  GNSS_GetPVTData(&GNSS_Handle);

		  loggerEmplaceU32(logBuffer, GPS_LON, GNSS_Handle.lon);
		  loggerEmplaceU32(logBuffer, GPS_LAT, GNSS_Handle.lat);
		  loggerEmplaceU32(logBuffer, GPS_SPD, GNSS_Handle.gSpeed);
		  logBuffer[GPS_FIX] = GNSS_Handle.fixType;
		  GPS_Timer = HAL_GetTick();
	  }

	  logBuffer[DRS] = drs;
	  logBuffer[TESTNO] = testNo;

	  static uint32_t usbTimeout = 0;
	  if(HAL_GetTick() - usbTimeout > 250) {
		  usbTimeout = HAL_GetTick();
		  adcEnable();
		  switch(usbBuffer[0]) {
		  case '0':
			  sprintf(msg, "AIN%c: %d\r\n", usbBuffer[0], eGetAnalog(&hspi4, 0).value);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case '1':
			  sprintf(msg, "AIN%c: %d\r\n", usbBuffer[0], eGetAnalog(&hspi4, 1).value);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case '2':
			  sprintf(msg, "AIN%c: %d\r\n", usbBuffer[0], eGetAnalog(&hspi4, 2).value);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case '3':
			  sprintf(msg, "AIN%c: %d\r\n", usbBuffer[0], eGetAnalog(&hspi4, 3).value);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case '4':
			  sprintf(msg, "AIN%c: %d\r\n", usbBuffer[0], eGetAnalog(&hspi4, 4).value);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case '5':
			  sprintf(msg, "AIN%c: %d\r\n", usbBuffer[0], eGetAnalog(&hspi4, 5).value);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case '6':
			  sprintf(msg, "AIN%c: %d\r\n", usbBuffer[0], eGetAnalog(&hspi4, 6).value);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case '7':
			  sprintf(msg, "AIN%c: %d\r\n", usbBuffer[0], eGetAnalog(&hspi4, 7).value);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case 's':
			  sprintf(msg, "FL: %d\tFR: %d\tRR: %d\tRL: %d\r\n", flsg, frsg, rrsg, rlsg);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case 'i':
			  sprintf(msg, "xAccel: %ld\tyAccel: %ld\tzAccel: %ld\r\n", xAccel, yAccel, zAccel);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case 'c':
			  sprintf(msg, "messages: %ld\tfifo full: %d\tfifo level: %ld\r\n", count, canFifoFull, HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0));
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case 'a':
			  sprintf(msg, "FBP: %d\tRBP: %d\tSTP: %d\tFLS: %d\tFRS: %d\tRRS: %d\tRLS: %d\r\n",
					  logBuffer[F_BRAKEPRESSURE] << 8 | logBuffer[F_BRAKEPRESSURE1],
					  logBuffer[R_BRAKEPRESSURE] << 8 | logBuffer[R_BRAKEPRESSURE1],
					  logBuffer[STEERING] << 8 | logBuffer[STEERING1],
					  logBuffer[FLSHOCK] << 8 | logBuffer[FLSHOCK1],
					  logBuffer[FRSHOCK] << 8 | logBuffer[FRSHOCK1],
					  logBuffer[RRSHOCK] << 8 | logBuffer[RRSHOCK1],
					  logBuffer[RLSHOCK] << 8 | logBuffer[RLSHOCK1]
			  );
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case 'w':
			  sprintf(msg, "(rtr/amb/rpm)\tFL: %.2f/%.2f/%d\tFR: %.2f/%.2f/%d\tRR: %.2f/%.2f/%d\tRL: %.2f/%.2f/%d\r\n",
					  mlx90614(flw.objTemp), mlx90614(flw.ambTemp), flw.rpm,
					  mlx90614(frw.objTemp), mlx90614(frw.ambTemp), frw.rpm,
					  mlx90614(rrw.objTemp), mlx90614(rrw.ambTemp), rrw.rpm,
					  mlx90614(rlw.objTemp), mlx90614(rlw.ambTemp), rlw.rpm
					  );
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
		  case 'm':
			  sprintf(msg, "current file: data%d.benji\ttest no: %d\r\n", currRunNo, testNo);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case 'p':
			  sprintf(msg, "current draw: %.2f mA\tbattery: %.2f V\r\n", ((float) ( (short) getCurrent(&hi2c4) )) * 1.25, ((float) getVoltage(&hi2c4)) * 1.25 / 1000.0);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case 't':
			  testNo++;
			  sprintf(msg, "incrementing test number! current test: %d\r\n", testNo);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  usbBuffer[0] = 'm';
			  break;
		  case 'g':
			  sprintf(msg, "fix: %d\tDay: %d-%d-%d\tTime: %d:%d:%d\tLon: %ld\tLat: %ld\r\n", GNSS_Handle.fixType,
				  GNSS_Handle.day, GNSS_Handle.month, GNSS_Handle.year,
				  GNSS_Handle.hour, GNSS_Handle.min, GNSS_Handle.sec,
				  GNSS_Handle.lon, GNSS_Handle.lat
				  );
		      CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  case 'b':
			  sprintf(msg, "brake fluid temperature: %d\r\n", brakeFluid);
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  default:
			  sprintf(msg, "no option selected\r\n");
			  CDC_Transmit_HS((uint8_t*) msg, strlen(msg));
			  break;
		  }
		  adcDisable();
	  }

	  uint32_t time = HAL_GetTick();
	  loggerEmplaceU32(logBuffer, TS, time);
	  if(f_write(&file, &logBuffer, sizeof(logBuffer), &wbytes) == FR_OK) {
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	  }
	  f_sync(&file);



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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 32;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 25;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 0;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 1;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 13;
  hfdcan3.Init.NominalTimeSeg2 = 2;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.MessageRAMOffset = 0;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.RxFifo0ElmtsNbr = 64;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
  canFilter.IdType = FDCAN_STANDARD_ID;
  canFilter.FilterIndex = 0;
  canFilter.FilterType = FDCAN_FILTER_RANGE;
  canFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  // accept all IDs
  canFilter.FilterID1 = 0x00;
  canFilter.FilterID2 = 0x7FF;
  canFilter.RxBufferIndex = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan3, &canFilter);

  if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
	{
	  Error_Handler();
	}

  if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_FULL, 0);
  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x60404E72;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x60404E72;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_ENABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 1;
  /* USER CODE BEGIN SDMMC1_Init 2 */
  HAL_SD_Init(&hsd1);
  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief UART9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART9_Init(void)
{

  /* USER CODE BEGIN UART9_Init 0 */

  /* USER CODE END UART9_Init 0 */

  /* USER CODE BEGIN UART9_Init 1 */

  /* USER CODE END UART9_Init 1 */
  huart9.Instance = UART9;
  huart9.Init.BaudRate = 38400;
  huart9.Init.WordLength = UART_WORDLENGTH_8B;
  huart9.Init.StopBits = UART_STOPBITS_1;
  huart9.Init.Parity = UART_PARITY_NONE;
  huart9.Init.Mode = UART_MODE_TX_RX;
  huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart9.Init.OverSampling = UART_OVERSAMPLING_16;
  huart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart9) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart9, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart9, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart9) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART9_Init 2 */

  /* USER CODE END UART9_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
