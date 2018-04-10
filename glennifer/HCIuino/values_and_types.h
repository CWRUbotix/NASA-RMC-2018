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
#define SABERTOOTH_BAUD 		(38400)
#define ROBOCLAW_BAUD 			(38400)
#define CMD_HEADER_SIZE			(2)
#define RPY_HEADER_SIZE			(2)
#define INSTRUCTION_LEN 		(3)
//#define HEADER_FIELDS			(3)
#define SENSOR_ID_SIZE      	(1)
#define SENSOR_DATA_SIZE 		(1)
#define MOTOR_ID_SIZE 			(1)
#define MOTOR_INSTRUC_SIZE 		(2)
#define NUM_MOTORS 				(9)
#define NUM_SENSORS 			(100)
#define ANLG_READ_RES 			(12)
#define ANLG_WRITE_RES 			(12)

//ODrive Stuff
#define PARAM_ENC_POS 			PARAM_FLOAT_ENCODER_PLL_POS
#define PARAM_ENC_VEL 			PARAM_FLOAT_ENCODER_PLL_VEL

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

// PIN NUMBERS & ADDRESSES
//Motor Control Boards
#define SABERTOOTH_0_SLCT 		(22)
#define SABERTOOTH_1_SLCT 		(23)
#define ROBOCLAW_0_ADDR 		(0x80)
#define ROBOCLAW_1_ADDR 		(0x81)
#define ROBOCLAW_2_ADDR 		(0x82)


////////////////////////////////////////////////////////////////////////////////
//
//  DEFINE TYPES
//
////////////////////////////////////////////////////////////////////////////////

//SENSOR STUFF
enum SensorHardware {
	SH_NONE,
	SH_BL_POT,		// BL := brushless motor
	SH_BL_ENC_VEL, 	//
	SH_BL_ENC_POS, 	//
	SH_BL_CUR,		// Motor Current
	SH_BR_POT,		// BR := brushed motor
	SH_BR_ENC,		// 
	SH_BR_CUR,		// Motor Current Sense
	SH_I2C_BAT,		// 
	SH_PIN_LIMIT,	// Limit switch
	SH_PIN_POT,		// Uses an ADC
	SH_LD_CELL
};

//SENSOR INFO
typedef struct SensorInfo{
	SensorHardware hardware;
	uint8_t  addr; 				// When hardware = SH_I2C_* or ...
	uint8_t  whichMotor; 		// Holds the ID of the motor if applicable
	int      whichPin; 			// 
	bool     is_reversed;		// When hardware = SH_PIN_LIMIT
	float    responsiveness = 1;// 1 = responsiveness
	uint16_t scale = 1; 		// 1 unless needed
	int16_t  storedVal; 		// replacing the sensor_storedVals array
	HX711*   loadCell; 			// if this happens to be a load cell
}SensorInfo;

//MOTOR STUFF
enum MotorHardware {
	MH_NONE,
	MH_BL_VEL,		// if ODrive
	MH_BL_POS,		// 
	MH_BL_BOTH,		// 
	MH_ST_PWM, 		// if Sabertooth
	MH_ST_VEL, 		//
	MH_ST_POS, 		//
	MH_RC_VEL, 		// if RoboClaw
	MH_RC_POS, 		//
	MH_RC_BOTH, 	//
	MH_ALL			// if All?
};

enum MCType{
	MC_NONE,
	MC_ODRIVE,
	MC_BRUSHED,
	MC_ROBOCLAW
};

typedef struct MCInfo {
	MCType type = MC_NONE; 		// default is NONE
	SabertoothSimplified* ST; 	// the sabertooth board object if applicable
	uint8_t selectPin; 			// slave select pin
	ODriveArduino* odrive; 		// if MC_ODRIVE
	RoboClaw* roboclaw;
	uint8_t addr = 0; 			// used if MC_ROBOCLAW
}MCInfo;

//MOTOR INFO
typedef struct MotorInfo{
	MotorHardware hardware = MH_NONE; // default is NONE
	MCInfo*  board; 		// motor controller board info
	uint8_t  addr; 			// 
	uint8_t  whichMotor; 	// motor 0 or 1 on the board?
	uint16_t scale = 1; 	// 1 unless needed
	int16_t  setPt = 0;		// set point for motor (rather that use an array)
	float    kp; 			// When hardware = MH_RC_POS or MC_RC_VEL
	float    ki; 			// When hardware = MH_RC_POS or MC_RC_VEL
	float    kd; 			// When hardware = MH_RC_POS or MC_RC_VEL
	uint32_t qpps; 			// When hardware = MH_RC_POS or MC_RC_VEL
	uint32_t deadband; 		// When hardware = MH_RC_POS
	uint32_t minpos; 		// When hardware = MH_RC_POS
	uint32_t maxpos; 		// When hardware = MH_RC_POS
	uint16_t  maxDuty = 16384; 	// for limiting output duty cycle
	uint32_t accel;
	uint16_t feedbackSensorID;
	float    saturation;
	uint32_t lastUpdateTime;// replaces the motor_lastUpdateTime array
	uint16_t lastError; 	// for tracking the derivative
	float    integral; 		// replaces the motor_integrals array
}MotorInfo;


////////////////////////////////////////////////////////////////////////////////
//
//  GLOBAL VARIABLES
//
////////////////////////////////////////////////////////////////////////////////
FAULT_T faults[DEFAULT_BUF_LEN];						// to hold faults that have occurred
byte faultIndex = 0;									// tracks the location of the last fault

SensorInfo 	sensor_infos	[DEFAULT_BUF_LEN] 	= {}; 	// All initialized to SH_NONE
MotorInfo 	motor_infos		[DEFAULT_BUF_LEN] 	= {}; 	// All initialized to MH_NONE
MCInfo      board_infos     [DEFAULT_BUF_LEN]   = {}; 	// Will be smaller later

int16_t motor_setpoints		[DEFAULT_BUF_LEN] 	= {0,0,0,0,1000,1000,1000,1000}; // All others initialized to 0
uint8_t sensor_lastLimitVals[DEFAULT_BUF_LEN]	= {}; 	// All initialized to 0
int16_t sensor_storedVals	[DEFAULT_BUF_LEN] 	= {}; 	// All initialized to 0
float 	motor_integrals		[DEFAULT_BUF_LEN] 	= {}; 	//All initialized to 0
int16_t motor_lastUpdateTime[DEFAULT_BUF_LEN] 	= {}; 	//All initialized to 0
bool 	stopped 								= true;	// default status is stopped

int lastTime = 0;
int debugging[5] = {};

// ODriveArduino odrive0(Serial1);
// ODriveArduino odrive1(Serial2);
// ODriveArduino odrive2(Serial3);

#endif
