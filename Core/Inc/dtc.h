#ifndef INC_DTC_H
#define INC_DTC_H

#include "adc.h"

extern const uint32_t DTC_CHECK_INTERVAL;
extern uint32_t DTC_PREV_CHECK_TIME;

//Author of these awful structs: Alex Rumer, the first year (Who let him touch the Datalogger code? ¯\_(ツ)_/¯ )
#pragma pack(push, 1) //Pushes storage boundary to single bit for best storage efficiency
// DTC Codes
typedef struct {
    uint32_t dtcCodes; // Single 32-bit integer to store 25 bits relating to the DTC state of each device
} dtc_code_handler;

// DTC Bitwise Macros for Updating the Code Status
#define SET_DTC(container, index)   ((container)->dtcCodes &= ~(1U << (index)))
#define CLEAR_DTC(container, index) ((container)->dtcCodes |= (1U << (index)))
#define CHECK_DTC(container, index) ((container)->dtcCodes & (1U << (index)))

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

//ADC Data
typedef struct{

    ADC_Result device;
    uint8_t init:1; //Limits Storage to a single bit address
    uint8_t errState:1; //Limits Storage to a single bit address
    uint8_t measures;
    uint8_t ADC_Code_Index:5; //Limits Storage to 5 bit addresses maximizing the value at 32
    uint8_t bufferIndex; //Index to store most recent measurement
    uint16_t *readBuffer; //Array to store the last n measurements
    uint32_t total; //Total of all measurements
    uint16_t avg; //Average of all measurements
    uint32_t timer; //Timer to check measurement consistency
    uint32_t timeout; //Time to wait before setting DTC
    uint8_t range;

} adc_dtc;
#pragma pack(pop) // Restores default STM32 Byte Padding

//Initialize Analog Device DTC values, defines where to store DTC code and how many times to measure the avg response time
void ADC_DTC_Init(adc_dtc *data, uint8_t measures, uint8_t index, uint8_t range, uint32_t timeout);

//Update the measurement state of the CAN DTC handler
void ADC_DTC_State_Update(adc_dtc *data);

//Update the ADC Error State
void ADC_DTC_Error_Update(adc_dtc *data, uint32_t time);

//Initialize CAN Device DTC values, defines where to store DTC code and how many times to measure the avg response time
void CAN_DTC_Init(can_dtc *data, uint8_t index, uint8_t measures, uint8_t percentage_over_allowed, uint8_t start_time);

//Update the measurement state of the CAN DTC handler
void CAN_DTC_State_Update(can_dtc *data, uint16_t msgTime);

//Update the Average Response Measurement of the CAN DTC handler
void CAN_DTC_Response_Update(can_dtc *data);

//Check if the CAN device has not responded lately, if not send an Error code to the DTC
void CAN_DTC_Error_Update(can_dtc *data, uint32_t time);

//Simplified checking logic for switch case checks
void CAN_DTC_Update_All(can_dtc *data, uint32_t time);

//Initialize all DTC handlers (Those that require initialization)
void DTC_Init(uint32_t start_time);

//Update Error State of All DTC devices
void DTC_Error_All(uint32_t time);

extern volatile dtc_code_handler DTC_Error_State_instance;
extern volatile dtc_code_handler* DTC_Error_State;

// Declare pointers to Wheel Board DTC Handlers
extern can_dtc *frwDTC;
extern can_dtc *flwDTC;
extern can_dtc *rrwDTC;
extern can_dtc *rlwDTC;

// Declare pointers to String Gauge DTC Handlers
extern can_dtc *flsDTC;
extern can_dtc *frsDTC;
extern can_dtc *rlsDTC;
extern can_dtc *rrsDTC;

// Declare pointer to IMU DTC Handler
extern can_dtc *imuDTC;

// Declare pointer to Brake and Throttle DTC Handler
extern can_dtc *brakeNthrottleDTC;

//Brake Pressure DTC Handler
extern adc_dtc *fBrakePress_DTC;
extern adc_dtc *rBrakePress_DTC;

//Shock DTC Handlers
extern adc_dtc *flShock_DTC;
extern adc_dtc *frShock_DTC;
extern adc_dtc *rrShock_DTC;
extern adc_dtc *rlShock_DTC;

//CAN Device DTC Indexes
extern const uint8_t DTC_Index_frWheelBoard;
extern const uint8_t DTC_Index_flWheelBoard;
extern const uint8_t DTC_Index_rrWheelBoard;
extern const uint8_t DTC_Index_rlWheelBoard;
extern const uint8_t DTC_Index_flStringGauge;
extern const uint8_t DTC_Index_frStringGauge;
extern const uint8_t DTC_Index_rlStringGauge;
extern const uint8_t DTC_Index_rrStringGauge;
extern const uint8_t DTC_Index_IMU;
extern const uint8_t DTC_Index_brakeNthrottle;

//GPS Device DTC Indexes
extern const uint8_t DTC_Index_GPS_0;
extern const uint8_t DTC_Index_GPS_1;

//ADC Device DTC Indexes
extern const uint8_t DTC_Index_fBrakePress;
extern const uint8_t DTC_Index_rBrakePress;
extern const uint8_t DTC_Index_steer;
extern const uint8_t DTC_Index_flShock;
extern const uint8_t DTC_Index_frShock;
extern const uint8_t DTC_Index_rlShock;
extern const uint8_t DTC_Index_rrShock;

#endif