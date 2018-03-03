#ifndef VALUES_TYPES_H_FILE
#define VALUES_TYPES_H_FILE

////////////////////////////////////////////////////////////////////////////////
//
//	PREPROCESSOR DEFINES
//
////////////////////////////////////////////////////////////////////////////////

//Command type codes
#define CMD_READ_SENSORS		(0x01)
#define CMD_SET_OUTPUTS			(0x02)
#define CMD_HCI_TEST			(0x03)
#define RPY_HCI_TEST 			(0xA5)


// Sizes & Number of things
#define DEFAULT_BUF_LEN 		(256)
#define NUM_SENSORS 			(256)
#define HCI_BAUD 				(9600)
#define ODRIVE_BAUD 			(115200)
#define CMD_HEADER_SIZE			(2)
#define RPY_HEADER_SIZE			(2)
#define INSTRUCTION_LEN 		(3)
//#define HEADER_FIELDS			(3)
#define SENSOR_ID_SIZE      	(1)
#define SENSOR_DATA_SIZE 		(1)
#define NUM_MOTORS 				(9)
#define MOTOR_ID_SIZE 			(1)
#define MOTOR_INSTRUC_SIZE 		(2)


// FAULT CODES
#define FAULT_T 				uint8_t
#define NO_FAULT				(1)
//Comms Faults
#define FAULT_FAILED_WRITE		(2)
#define FAULT_INCOMPLETE_HEADER (3)
#define FAULT_CORRUPTED_HEADER 	(4)
#define FAULT_INCOMPLETE_BODY 	(5)
#define FAULT_CORRUPTED_BODY 	(6)

//Logging faults
#define FAULT_LOG_FULL			(7)

//Motor Faults



////////////////////////////////////////////////////////////////////////////////
//
//  DEFINE TYPES
//
////////////////////////////////////////////////////////////////////////////////

//SENSOR STUFF
enum SensorHardware {
	SH_NONE,
	SH_BL_POT,		// BL := brushless motor
	SH_BL_ENC,		//
	SH_BL_CUR,		// Motor Current
	SH_BR_POT,		// BR := brushed motor
	SH_BR_ENC,		// 
	SH_BR_CUR,		// Motor Current Sense
	SH_I2C_BAT,		// 
	SH_PIN_LIMIT,	// Limit switch
	SH_PIN_POT		// Uses an ADC
};

//SENSOR INFO
typedef struct SensorInfo{
	SensorHardware hardware;
	uint8_t addr; 			// When hardware = SH_I2C_* or ...
	uint8_t whichMotor; 	// When hardware = SH_RC_*
	int whichPin; 			// 
	bool is_reversed;		// When hardware = SH_PIN_LIMIT
	float responsiveness;	// 1 = responsiveness
	uint16_t scale; 		// 1 unless needed
	int16_t storedVal; 		// replacing the sensor_storedVals array
}SensorInfo;

//MOTOR STUFF
enum MotorHardware {
	MH_NONE,
	MH_BL_UART, 	// BL := brushless
	MH_BL_VEL,		// 
	MH_BL_POS,		// 
	MH_BL_POS_BOTH,	// 
	MH_BR_PWM, 		// BR := brushed
	MH_BR_POS,		// 
	MH_BR_PWM_BOTH,	// 
	MH_PIN_PWM,		// 
	MH_ALL			// 
};

enum CtrlrType {
	CTRL_BL,
	CTRL_BR
};

//MOTOR INFO
typedef struct MotorInfo{
	MotorHardware hardware = MH_NONE; // default is NONE
	uint8_t addr; 			// 
	uint8_t whichMotor;		// if brushless motors
	uint8_t whichCtrlr; 	// identify the controller
	uint8_t PWMpin; 		// if MH_BR_PWM
	uint16_t scale = 1; 	// 1 unless needed
	uint16_t setPt; 		// set point for motor (rather that use an array)
	float kp; 				// When hardware = MH_RC_POS or MC_RC_VEL
	float ki; 				// When hardware = MH_RC_POS or MC_RC_VEL
	float kd; 				// When hardware = MH_RC_POS or MC_RC_VEL
	uint32_t qpps; 			// When hardware = MH_RC_POS or MC_RC_VEL
	uint32_t deadband; 		// When hardware = MH_RC_POS
	uint32_t minpos; 		// When hardware = MH_RC_POS
	uint32_t maxpos; 		// When hardware = MH_RC_POS
	uint32_t accel;
	uint16_t feedbackSensorID;
	float saturation;
	int16_t lastUpdateTime;	// replaces the motor_lastUpdateTime array
	float integral; 		// replaces the motor_integrals array
}MotorInfo;


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

int lastTime = 0;
int debugging[5] = {};

#endif