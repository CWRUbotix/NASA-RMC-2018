////////////////////////////////////////////////////////////////////////////////
//
//	values_and_types.h
// 		
//		This file covers:
//		• Preprocessor defines
// 		• Type Definitions
// 		• Global Variables
//
////////////////////////////////////////////////////////////////////////////////

#ifndef VALUES_TYPES_H_FILE
#define VALUES_TYPES_H_FILE

////////////////////////////////////////////////////////////////////////////////
//	PREPROCESSOR DEFINES
////////////////////////////////////////////////////////////////////////////////
//Command type codes
#define CMD_READ_SENSORS		(0x01)
#define CMD_SET_OUTPUTS			(0x02)
#define CMD_HCI_TEST			(0x03)
#define CMD_SET_PID 			(0x04)
#define RPY_HCI_TEST 			(0xA5)


// Sizes & Number of things
#define DEFAULT_BUF_LEN 		(256)
#define NUM_SENSORS 			(256)
#define NUM_MOTORS 				(12)
#define NUM_LIM_SWITCHES 		(9)
#define NUM_EXC_LIMS 			(3)
#define NUM_EXC_ROT_LIMS 		(2)
#define NUM_DEP_LIMS 			(4)
#define HCI_BAUD 				(115200)
#define SABERTOOTH_BAUD 		(38400)
#define CMD_HEADER_SIZE			(2)
#define RPY_HEADER_SIZE			(2)
#define INSTRUCTION_LEN 		(3)
#define SENSOR_ID_SIZE      	(1)
#define SENSOR_DATA_SIZE 		(1)
#define MOTOR_ID_SIZE 			(1)
#define MOTOR_INSTRUC_SIZE 		(2)
#define ANLG_READ_RES 			(12)
#define ANLG_WRITE_RES 			(12)
#define DFLT_MAX_DELTA 			(200)
#define DGTL_WRITE_DELAY 		(2)
#define LOW_PASS_ARRAY_SIZE 	(2)
#define HX711_TIMEOUT 			(10)

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
#define SABERTOOTH_ROT_M1 		(22)
#define SABERTOOTH_ROT_M2 		(23)
#define SABERTOOTH_TRANS_M1 	(53)
#define E_STOP_PIN 				(2)
#define QUAD_ENC_READER_ADDR 	(0x14)
#define ENCODER_MODE 			((uint8_t) 128)
#define FRONT_PORT_MTR_ID 		(0)
#define REAR_PORT_MTR_ID 		(2)
#define FRONT_STARBOARD_MTR_ID 	(1)
#define REAR_STARBOARD_MTR_ID 	(3)
#define DEP_WINCH_MOTOR_ID 		(4)
#define PORT_LIN_ACT_ID 		(6)
#define STARBOARD_LIN_ACT_ID 	(7)
#define LIN_ACT_KP 				(1.5)
#define LIN_ACT_KI 				(0.000000001)
#define EXC_TRANSLATION_KP 		(1.2)
#define EXC_TRANSLATION_KI 		(0.000000002)
#define KP_INC 					(0.1)
#define KI_INC 					(0.000000001)
#define DRIVE_KP 				(0.036)
#define DRIVE_KI 				(0.0000000065)

// ESC MOTOR CONTROL VALUES
#define RELAY_RISE_FALL_TIME 	(10)
#define PWM_PIN_ESC_1 			(40)
#define REV_PIN_ESC_1 			(51)
#define PWM_PIN_ESC_2 			(42)
#define REV_PIN_ESC_2 			(49)
#define PWM_PIN_ESC_3 			(44)
#define REV_PIN_ESC_3 			(43)
#define PWM_PIN_ESC_4 			(46)
#define REV_PIN_ESC_4 			(47)
#define PWM_PIN_ESC_5 			(48)
#define REV_PIN_ESC_5 			(41)
#define PWM_PIN_ESC_6     		(50)
#define REV_PIN_ESC_6     		(45)
#define ESC_SPEED_MIN 			(1000)
#define ESC_SPEED_MAX 			(2000)


////////////////////////////////////////////////////////////////////////////////
//
//  DEFINE TYPES
//
////////////////////////////////////////////////////////////////////////////////

enum SubSystem{
	LOCO_SYS,
	EXC_SYS,
	DEP_SYS,
	ALL_SYS
};

//SENSOR STUFF
enum SensorHardware {
	SH_NONE,
	SH_QUAD_VEL,
	SH_QUAD_POS,
	SH_BL_POT,		// BL := brushless motor
	SH_BL_ENC_VEL, 	//
	SH_BL_ENC_POS, 	//
	SH_RC_ENC_VEL, 	//
	SH_RC_ENC_POS, 	//
	SH_BL_CUR,		// Motor Current
	SH_BR_POT,		// BR := brushed motor
	SH_BR_ENC,		// 
	SH_BR_CUR,		// Motor Current Sense
	SH_I2C_BAT,		// 
	SH_PIN_LIMIT,	// Limit switch
	SH_PIN_POT,		// Uses an analog pin
	//SH_POT_POS, 	// For determining position using pots
	SH_LD_CELL
};

//SENSOR INFO
typedef struct SensorInfo{
	SensorHardware hardware = SH_NONE;
	uint8_t  addr; 				// When hardware = SH_I2C_* or ...
	uint8_t  whichMotor = 0; 	// Holds the ID of the motor if applicable
	int      whichPin; 			// 
	bool     is_reversed = false;// When hardware = SH_PIN_LIMIT
	float    responsiveness = 1;// 1 = responsiveness
	uint16_t scale = 1; 		// 1 unless needed
	int32_t  baseline;
	int16_t  storedVal; 		// replacing the sensor_storedVals array
	int16_t* prev_values; 		// 
	int16_t  val_at_max; 		// when this is a linear pot or similar
	int16_t  val_at_min; 		// ^ ^ ^
	uint32_t lastUpdateTime; 	// 
	int8_t   mtr_dir_if_triggered = 0; // will be set to either 1 or -1 if being used
	uint8_t  array_index;
	uint32_t last_pos = 0; 		// used for encoders
}SensorInfo;

//MOTOR STUFF
enum MotorHardware {
	MH_NONE,
	MH_BL_VEL,		// if ESC
	MH_BL_POS,		// 
	MH_BL_BOTH,		// 
	MH_ST_PWM, 		// if Sabertooth
	MH_ST_VEL, 		//
	MH_ST_POS, 		//
	MH_RC_VEL, 		// if RoboClaw
	MH_RC_POS, 		//
	MH_RC_BOTH, 	//
	MH_LOOKY,
	MH_BL_OPEN_LOOP,
	MH_ALL			// if All?
};

enum MCType{
	MC_NONE,
	MC_YEP,
	MC_SABERTOOTH
};

typedef struct MCInfo {
	MCType type = MC_NONE; 		// default is NONE
	SabertoothSimplified* ST; 	// the sabertooth board object if applicable
	uint8_t selectPin; 			// slave select pin
	uint8_t addr; 				// 
	ESC* esc; 					// if using the brushless controller
	uint8_t  dir_relay_pin; 	// used to control direction
}MCInfo;

//MOTOR INFO
typedef struct MotorInfo{
	MotorHardware hardware = MH_NONE; // default is NONE
	MCInfo*  board; 			// motor controller board info
	SensorInfo* encoder; 		// pointer to this motor's encoder
	uint8_t  subsys = ALL_SYS; 	// flag to set the subsystem
	uint8_t  whichMotor; 		// motor 0 or 1 on the board?
	uint8_t  whichPin = 0; 		// if it's a sabertooth
	bool     is_reversed = false;
	uint16_t scale = 1; 		// 1 unless needed
	int16_t  setPt = 0;			// set point for motor (rather that use an array)
	int32_t  target_vel = 0; 	//
	int32_t  target_pos = 0;
	int16_t  lastSet = 0; 		//
	float    kp; 				// When hardware = MH_RC_POS or MC_RC_VEL
	float    ki; 				// When hardware = MH_RC_POS or MC_RC_VEL
	float    kd; 				// When hardware = MH_RC_POS or MC_RC_VEL
	uint32_t qpps; 				// When hardware = MH_RC_POS or MC_RC_VEL
	uint32_t deadband; 			// When hardware = MH_RC_POS
	uint32_t center = 1500; 	// center from which to add/sub deadband
	int16_t  max_pwr = 500; 	// for stopping out-of-sync actuation
	float    current_pwr 	= 0;// 
	float    last_pwr 		= 0;// 
	uint16_t margin = 10; 		// how far from set-point is acceptable?
	uint32_t minpos; 			// When hardware = MH_RC_POS
	uint32_t maxpos; 			// When hardware = MH_RC_POS
	uint16_t maxDuty = 16384; 	// for limiting output duty cycle
	int16_t  max_delta = 0;		// for ramp-up function
	uint8_t  feedbackSensorID;	//
	float    saturation; 		//
	uint32_t lastUpdateTime; 	// replaces the motor_lastUpdateTime array
	uint32_t safe_dt = 1000; 	// safe time to wait for the motor to stop
	uint16_t lastError; 		// for tracking the derivative
	float    integral; 			// replaces the motor_integrals array
	float    integral_max = 10000.0;
	bool     is_stopped = false;// to stop if we hit a limit switch
	uint8_t  looky_id;
	SensorInfo** limits; 		// points to array that holds limit switches
	uint8_t  num_limits; 		// number of limit switches in the above array
}MotorInfo;

int sign(int val){
	return (0 < val) - (val < 0);
}

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
SensorInfo* limit_switches 	[DEFAULT_BUF_LEN] 	= {}; 	// iterate thru & check for collisions
ESC* 		yep_escs 		[DEFAULT_BUF_LEN] 	= {};

int16_t motor_setpoints		[DEFAULT_BUF_LEN] 	= {0,0,0,0,1000,1000,1000,1000}; // All others initialized to 0
uint8_t sensor_lastLimitVals[DEFAULT_BUF_LEN]	= {}; 	// All initialized to 0
int16_t sensor_storedVals	[DEFAULT_BUF_LEN] 	= {}; 	// All initialized to 0
float 	motor_integrals		[DEFAULT_BUF_LEN] 	= {}; 	//All initialized to 0
int16_t motor_lastUpdateTime[DEFAULT_BUF_LEN] 	= {}; 	//All initialized to 0
int8_t  encoder_values      [8] 				= {}; 	// for values from encoder board
SensorInfo* exc_limits      [NUM_EXC_LIMS] 		= {};
SensorInfo* exc_rot_limits  [NUM_EXC_ROT_LIMS] 	= {};
SensorInfo* dep_limits 		[NUM_DEP_LIMS] 		= {};

int16_t port_side_low_pass_arr[LOW_PASS_ARRAY_SIZE] = {};
int16_t stbd_side_low_pass_arr[LOW_PASS_ARRAY_SIZE] = {};
int16_t translate_low_pass_arr[LOW_PASS_ARRAY_SIZE] = {};
int16_t loady_boi_1_lo_pass_arr[LOW_PASS_ARRAY_SIZE]= {};
int16_t loady_boi_2_lo_pass_arr[LOW_PASS_ARRAY_SIZE]= {};
bool 	stopped 								= true;	// default status is stopped
uint8_t e_stop_state 							= LOW;
uint8_t e_stop_state_last 						= LOW;

int lastTime = 0;

uint32_t loops 									= 0;

// CREATE ESC OBJECTS
ESC ESC_1 (PWM_PIN_ESC_1, ESC_SPEED_MIN, ESC_SPEED_MAX, 500);                 // ESC_Name (ESC PIN, Minimum Value, Maximum Value, Default Speed, Arm Value)
ESC ESC_2 (PWM_PIN_ESC_2, ESC_SPEED_MIN, ESC_SPEED_MAX, 500);
ESC ESC_3 (PWM_PIN_ESC_3, ESC_SPEED_MIN, ESC_SPEED_MAX, 500);
ESC ESC_4 (PWM_PIN_ESC_4, ESC_SPEED_MIN, ESC_SPEED_MAX, 500);
ESC ESC_5 (PWM_PIN_ESC_5, ESC_SPEED_MIN, ESC_SPEED_MAX, 500);
ESC ESC_6 (PWM_PIN_ESC_6, ESC_SPEED_MIN, ESC_SPEED_MAX, 500);

#endif
