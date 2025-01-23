#include "dtc.h"
#include <stdlib.h>

const uint32_t DTC_CHECK_INTERVAL = 20;
uint32_t DTC_PREV_CHECK_TIME = 0;

//******************************************************************* 
//Defining the DTC Storage Index for each recorded DTC device

//CAN Device DTC Indexes
const uint8_t DTC_Index_frWheelBoard = 0;
const uint8_t DTC_Index_flWheelBoard = 1;
const uint8_t DTC_Index_rrWheelBoard = 2;
const uint8_t DTC_Index_rlWheelBoard = 3;
const uint8_t DTC_Index_flStringGauge = 11;
const uint8_t DTC_Index_frStringGauge = 12;
const uint8_t DTC_Index_rlStringGauge = 13;
const uint8_t DTC_Index_rrStringGauge = 14;
const uint8_t DTC_Index_IMU = 15;
const uint8_t DTC_Index_brakeNthrottle = 16;

//GPS Device DTC Indexes
const uint8_t DTC_Index_GPS_0 = 17;
const uint8_t DTC_Index_GPS_1 = 18;

//ADC Device DTC Indexes
const uint8_t DTC_Index_fBrakePress = 4;
const uint8_t DTC_Index_rBrakePress = 5;
const uint8_t DTC_Index_steer = 6;
const uint8_t DTC_Index_flShock = 7;
const uint8_t DTC_Index_frShock = 8;
const uint8_t DTC_Index_rlShock = 9;
const uint8_t DTC_Index_rrShock = 10;

//*******************************************************************

volatile dtc_code_handler DTC_Error_State_instance = {0};
volatile dtc_code_handler* DTC_Error_State = &DTC_Error_State_instance;


//Initialize the DTC handlers and allocate memory
can_dtc frwDTC_instance, flwDTC_instance, rrwDTC_instance, 
rlwDTC_instance, flsDTC_instance, frsDTC_instance, rlsDTC_instance, 
rrsDTC_instance, imuDTC_instance, brakeNthrottleDTC_instance;

//Wheel Board DTC Handlers
can_dtc *frwDTC = &frwDTC_instance;
can_dtc *flwDTC = &flwDTC_instance;
can_dtc *rrwDTC = &rrwDTC_instance;
can_dtc *rlwDTC = &rlwDTC_instance;

//String Gauge DTC Handlers
can_dtc *flsDTC = &flsDTC_instance;
can_dtc *frsDTC = &frsDTC_instance;
can_dtc *rlsDTC = &rlsDTC_instance;
can_dtc *rrsDTC = &rrsDTC_instance;

//IMU DTC Handler
can_dtc *imuDTC = &imuDTC_instance;

//Brake and Throttle DTC Handler
can_dtc *brakeNthrottleDTC = &brakeNthrottleDTC_instance;

//Initialize CAN Device DTC values, defines where to store DTC code and how many times to measure the avg response time
void CAN_DTC_Init(can_dtc *data, uint8_t index, uint8_t measures, uint16_t percentage_over_allowed, uint32_t start_time){
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
        data->timeBuffer[i] = start_time;
    }
	return;
}

//Update the measurement state of the CAN DTC handler
void CAN_DTC_State_Update(can_dtc *data, uint32_t msgTime){
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

	uint32_t total = 0;
	for(int i=0; i<data->measures; i++){
		total += data->timeBuffer[i];
	}
	//Error and Value Checking
	if(data->init){
		//Calculate Average Response Time
		data->avgResponse = total / data->measures;
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
	else{
		CLEAR_DTC(DTC_Error_State, data->DTC_Code_Index);
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

//Initialize all DTC handlers (Those that require initialization)
void DTC_Init(uint32_t start_time){
  //Wheel Board DTC Handlers
	CAN_DTC_Init(frwDTC, DTC_Index_frWheelBoard, 10, 25, start_time);
	CAN_DTC_Init(flwDTC, DTC_Index_flWheelBoard, 10, 25, start_time);
	CAN_DTC_Init(rrwDTC, DTC_Index_rrWheelBoard, 10, 200, start_time);
	CAN_DTC_Init(rlwDTC, DTC_Index_rlWheelBoard, 10, 200, start_time);

  //String Gauge DTC Handlers
	CAN_DTC_Init(flsDTC, DTC_Index_flStringGauge, 10, 25, start_time);
	CAN_DTC_Init(frsDTC, DTC_Index_frStringGauge, 10, 25, start_time);
	CAN_DTC_Init(rlsDTC, DTC_Index_rlStringGauge, 10, 25, start_time);
	CAN_DTC_Init(rrsDTC, DTC_Index_rrStringGauge, 10, 25, start_time);

	//IMU DTC Handler
	CAN_DTC_Init(imuDTC, DTC_Index_IMU, 10, 500, start_time);

	//Brake and Throttle DTC Handler
	CAN_DTC_Init(brakeNthrottleDTC, DTC_Index_brakeNthrottle, 10, 25, start_time);


	for(int i=0; i<32; i++)CLEAR_DTC(DTC_Error_State, i);
	return;
}

void DTC_Error_All(uint32_t time){
	//CAN DTC
	CAN_DTC_Error_Update(frwDTC, time);
	CAN_DTC_Error_Update(flwDTC, time);
	CAN_DTC_Error_Update(rrwDTC, time);
	CAN_DTC_Error_Update(rlwDTC, time);
	CAN_DTC_Error_Update(flsDTC, time);
	CAN_DTC_Error_Update(frsDTC, time);
	CAN_DTC_Error_Update(rlsDTC, time);
	CAN_DTC_Error_Update(rrsDTC, time);
	CAN_DTC_Error_Update(imuDTC, time);
	CAN_DTC_Error_Update(brakeNthrottleDTC, time);

	return;
}
