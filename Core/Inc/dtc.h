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
	uint16_t percentOver; //Store the percentage of avg response time over allowed before throwing an error
	uint32_t *timeBuffer;


}can_dtc; //This name needs work I know...


//Initialize CAN Device DTC values, defines where to store DTC code and how many times to measure the avg response time
void CAN_DTC_Init(can_dtc *data, uint8_t index, uint8_t measures, uint16_t percentage_over_allowed, uint32_t start_time);

//Update the measurement state of the CAN DTC handler
void CAN_DTC_State_Update(can_dtc *data, uint32_t msgTime);

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

extern can_dtc *shifterDTC;

// DTC Index Enum
typedef enum {
    // CAN Device DTC Indexes (0-3)
    DTC_Index_frWheelBoard,
    DTC_Index_flWheelBoard,
    DTC_Index_rrWheelBoard,
    DTC_Index_rlWheelBoard,
    
    // ADC Device DTC Indexes (4-10)
    DTC_Index_fBrakePress,
    DTC_Index_rBrakePress,
    DTC_Index_steer,
    DTC_Index_flShock,
    DTC_Index_frShock,
    DTC_Index_rlShock,
    DTC_Index_rrShock,
    
    // String Gauge DTC Indexes (11-14)
    DTC_Index_flStringGauge,
    DTC_Index_frStringGauge,
    DTC_Index_rlStringGauge,
    DTC_Index_rrStringGauge,
    
    // IMU and Brake/Throttle DTC Indexes (15-16)
    DTC_Index_IMU,
    DTC_Index_brakeNthrottle,
    
    // GPS Device DTC Indexes (17-18)
    DTC_Index_GPS_0,
    DTC_Index_GPS_1,
    
    // Shifter DTC Index (19)
    DTC_Index_Shifter
} DTC_Index_t;


#endif
