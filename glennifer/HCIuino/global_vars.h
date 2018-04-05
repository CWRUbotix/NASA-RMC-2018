#ifndef GLOBAL_VARS
#define GLOBAL_VARS

////////////////////////////////////////////////////////////////////////////////
//
//  GLOBAL VARIABLES
//
////////////////////////////////////////////////////////////////////////////////
FAULT_T faults[DEFAULT_BUF_LEN];						// to hold faults that have occurred
byte faultIndex = 0;									// tracks the location of the last fault

SensorInfo sensor_infos[DEFAULT_BUF_LEN] 		= {}; 	// All initialized to SH_NONE
MotorInfo motor_infos[DEFAULT_BUF_LEN] 			= {}; 	// All initialized to MH_NONE

int16_t motor_setpoints[DEFAULT_BUF_LEN] 		= {0,0,0,0,1000,1000,1000,1000}; // All others initialized to 0
uint8_t sensor_lastLimitVals[DEFAULT_BUF_LEN]	= {}; 	// All initialized to 0
int16_t sensor_storedVals[DEFAULT_BUF_LEN] 		= {}; 	// All initialized to 0
float motor_integrals[DEFAULT_BUF_LEN] 			= {}; 	//All initialized to 0
int16_t motor_lastUpdateTime[DEFAULT_BUF_LEN] 	= {}; 	//All initialized to 0
bool stopped 									= true;	// default status is stopped

#endif