////////////////////////////////////////////////////////////////////////////////
//
//	PREPROCESSOR INCLUDES
//
////////////////////////////////////////////////////////////////////////////////
//#include <ODriveArduino.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//
//	PREPROCESSOR DEFINES
//
////////////////////////////////////////////////////////////////////////////////

//Command type codes
#define CMD_READ_SENSORS		0x01
#define CMD_SET_OUTPUTS			0x02
#define CMD_HCI_TEST			(0x03)
#define RPY_HCI_TEST 			(0xA5)

// Sizes & Number of things
#define SENSOR_DATA_SIZE 		(1)
#define SENSOR_ID_SIZE      (2)
#define MOTOR_ID_SIZE 			(1)
#define CMD_HEADER_SIZE			(2)
#define RPY_HEADER_SIZE			(2)
#define HEADER_FIELDS			(3)
#define INSTRUCTION_LEN 		(3)
#define DEFAULT_BUF_LEN 		(256)
#define SENSORE_ID_SIZE 		(1)
#define MOTOR_ID_SIZE 			(1)
#define UART_BAUD 				(115200)

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
	SH_BL_CUR,		//
	SH_BR_POT,		// BR := brushed motor
	SH_BR_ENC,		//
	SH_BR_CUR,		//
	SH_I2C_BAT,		//
	SH_PIN_LIMIT,	//
	SH_PIN_POT		//
};

//SENSOR INFO
typedef struct SensorInfo{
	SensorHardware hardware;
	uint8_t addr; 			// When hardware = SH_I2C_* or ...
	uint8_t whichMotor; 	// When hardware = SH_RC_*
	uint8_t whichPin; 		// When hardware = SH_PIN_*
	bool is_reversed;		// When hardware = SH_PIN_LIMIT
	float responsiveness;	// 1 is perfect responsiveness??
	uint16_t scale; 		// 1 unless needed
}SensorInfo;

//MOTOR STUFF
enum MotorHardware {
	MH_NONE,
	MH_BL_PWM, 		// BL := brushless
	MH_BL_VEL,		//
	MH_BL_POS,		//
	MH_BL_POS_BOTH,	//
	MH_BR_PWM, 		// BR := brushed
	MH_BR_POS,		//
	MH_BR_PWM_BOTH,	//
	MH_PIN_PWM,		//
	MH_ALL			//
};

//MOTOR INFO
typedef struct MotorInfo{
	MotorHardware hardware = MH_NONE; // default is NONE
	uint8_t addr; 			// 
	uint8_t whichMotor;		// 
	uint16_t scale; 		// 1 unless needed
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


////////////////////////////////////////////////////////////////////////////////
//
//  FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////

FAULT_T log_fault(FAULT_T fault, byte* rpy){
	//form a reply with relevant data regarding the fault

	if(faultIndex < 255){
		faults[faultIndex++] = fault;
		return NO_FAULT;
	}else{
		faults[faultIndex]   = fault;
		return FAULT_LOG_FULL;
	}
}

void clear_fault_log(){
//	FAULT_T tmp[256] = {}; 	// allocate an empty buffer
//	faults 			 = tmp;
}

// Return the most recent fault
FAULT_T popLastFault(){
	FAULT_T retVal = faults[faultIndex]; 	// stash current fault value to return
	faults[faultIndex] = 0;					// 0 has no meaning

	if(faultIndex > 0){
		faultIndex--;			
	}
	return retVal;

}


////////////////////////////////////////////////////////////////////////////////
void setup_sensors(){
	// FOREACH SENSOR

}
////////////////////////////////////////////////////////////////////////////////
void setup_motors(){
	// FOREACH MOTOR
}
////////////////////////////////////////////////////////////////////////////////
void setup_comms() {
  SerialUSB.begin(9600);
  Serial.begin(9600);
}

////////////////////////////////////////////////////////////////////////////////
//		• read & verify data from the client
//		• populate the array argument with this data
////////////////////////////////////////////////////////////////////////////////
FAULT_T hciRead(byte * cmd){
	uint8_t type;
	uint8_t bodyLen;
	uint8_t retval;

	retval = SerialUSB.readBytes(cmd, CMD_HEADER_SIZE);	// populate the array

	// proceed to check the cmd array for issues
	if(retval != CMD_HEADER_SIZE){
		return FAULT_INCOMPLETE_HEADER;
	}
	if(!cmd_check_head(cmd, bodyLen)){
		return FAULT_CORRUPTED_HEADER;
	}

	bodyLen = cmd_body_len(cmd);
	retval = SerialUSB.readBytes(cmd + CMD_HEADER_SIZE, bodyLen);

	if(retval < bodyLen){
		return FAULT_INCOMPLETE_BODY;
	}
	if (!cmd_check_body(cmd, bodyLen)) {
		return FAULT_CORRUPTED_BODY;
	}

	// success!
	return NO_FAULT;
}


////////////////////////////////////////////////////////////////////////////////
bool cmd_check_head(byte * cmd, uint8_t bodyLen) {
	return true;
}
////////////////////////////////////////////////////////////////////////////////
uint16_t cmd_body_len(byte* cmd){
	return cmd[1];	// length is the second byte
}
////////////////////////////////////////////////////////////////////////////////
bool cmd_check_body(byte * cmd, uint8_t bodyLen){
	return (bodyLen % INSTRUCTION_LEN)==0;
}
////////////////////////////////////////////////////////////////////////////////
uint8_t cmd_type(byte cmd[]) {
  return cmd[0];
}
////////////////////////////////////////////////////////////////////////////////
uint8_t cmd_sense_num_sensors(byte* cmd){
	return cmd_body_len(cmd);
}


////////////////////////////////////////////////////////////////////////////////
//		• takes empty byte array as an argument 
//		• populates this with relevant information to respond to the client
////////////////////////////////////////////////////////////////////////////////
FAULT_T hciAnswer(byte* cmd, byte* rpy){
	//byte rpy[DEFAULT_BUF_LEN];
	bool success 	= false;
	uint8_t size 	= RPY_HEADER_SIZE;
	uint8_t bodyLen = 0;
	uint8_t retval 	= 0;
	FAULT_T fault 	= FAULT_FAILED_WRITE;

	uint8_t type = cmd_type(cmd);
	rpy_init(rpy, type);
	uint8_t num = 0;
	uint8_t i, ID, index;
	switch(type){
		case CMD_HCI_TEST:
			rpy[RPY_HEADER_SIZE] = RPY_HCI_TEST; 	// data goes immediately after header
			bodyLen++;
			break;
		case CMD_READ_SENSORS:
			int16_t value;
			num = cmd_sense_num_sensors(cmd);
			for(i = 0; i<num; i++){
				ID    = cmd[(3*i)+CMD_HEADER_SIZE];
				read_sensor(ID, &value);
				rpy[(3*i)+RPY_HEADER_SIZE]   = ID;
				rpy[(3*i)+RPY_HEADER_SIZE+1] = (uint8_t)(value >> 8);
				rpy[(3*i)+RPY_HEADER_SIZE+2] = (uint8_t)(value & 0xFF);
				bodyLen += 3;
			}
			break;
		case CMD_SET_OUTPUTS:
			num = cmd_body_len(cmd)/3;
			uint16_t setPt;
			for(i = 0; i<num; i++){
				ID    = cmd[(3*i)+CMD_HEADER_SIZE];
				setPt = motor_infos[ID].setPt;
				rpy[RPY_HEADER_SIZE+i*3]   = ID;
				rpy[RPY_HEADER_SIZE+i*3+1] = (uint8_t)(setPt >> 8);
				rpy[RPY_HEADER_SIZE+i*3+2] = (uint8_t)(setPt & 0xFF);
				bodyLen+=3;
			}
			break;

	}

	rpy_set_len(rpy, bodyLen); 	// we only want to consider body length
	size 	+= bodyLen;
	retval 	= SerialUSB.write(rpy, size);
	success = (retval == size);
	if(success){
		//clear_fault_log();
		fault = NO_FAULT;
	}else{
		fault = FAULT_FAILED_WRITE;
	}

	return fault;
}

////////////////////////////////////////////////////////////////////////////////
//		• takes reply bytes as an array argument 
//		• write these bytes to the client
////////////////////////////////////////////////////////////////////////////////
FAULT_T hciWrite(byte rpy[]){
	uint16_t len 	= rpy_len(rpy);	// DIFF from last year
	FAULT_T fault	= FAULT_FAILED_WRITE;
	uint8_t retval 	= SerialUSB.write(rpy,len+2);
	
	if (retval == len+2) {
    	fault = NO_FAULT;
	}else{
		fault = FAULT_FAILED_WRITE;
	}

	return fault;
}


////////////////////////////////////////////////////////////////////////////////
uint8_t rpy_len(byte * rpy){
	// uint16_t ret;
	// ret += rpy[0];
	// ret << 8;
	// ret += rpy[1];
	return rpy[1];
}
////////////////////////////////////////////////////////////////////////////////
void rpy_init(byte* rpy, uint8_t type){
	rpy[0] = type; 	// first byte is type
	rpy[1] = 0; 	// explicitly initialize to 0
}
////////////////////////////////////////////////////////////////////////////////
void rpy_set_len(byte* rpy, uint8_t length){
	rpy[1] = length;
}
////////////////////////////////////////////////////////////////////////////////
uint8_t cmd_sense_sensor_id(byte* cmd, uint8_t i){
	uint8_t index = CMD_HEADER_SIZE; 	// index of the first byte of the body
	index += (i * SENSOR_DATA_SIZE); 	// index of the i-th sensor
	return cmd[index]; 					// return the id
}

////////////////////////////////////////////////////////////////////////////////
// 	@param ID:	takes a sensor ID
// 	@param val:	ptr to the 16-bit signed int where the sensor data will go
////////////////////////////////////////////////////////////////////////////////
FAULT_T read_sensor(uint8_t ID, int16_t* val){
	SensorInfo sensor_info 	= sensor_infos[ID];
	uint8_t status 			= 0;
	bool valid 				= false;
	int32_t val32 			= 0;
	int16_t dummy 			= 0;
	int16_t readVal 		= 0;

	switch (sensor_info.hardware) {
		case SH_PIN_LIMIT:
			if(sensor_info.is_reversed){
				*val = !digitalRead(sensor_info.whichPin);
			}else{
				*val = digitalRead(sensor_info.whichPin);
			}
			break;

		case SH_PIN_POT:
			readVal 				= (int16_t)analogRead(sensor_info.whichPin) / sensor_info.scale;
			sensor_storedVals[ID] 	= (sensor_storedVals[ID] * (1 - sensor_info.responsiveness)) + (readVal * sensor_info.responsiveness);
			*val 					= sensor_storedVals[ID];
			break;
	}
	return NO_FAULT;
}


////////////////////////////////////////////////////////////////////////////////
//	maintain_motors(byte* cmd)
//	if success is true:
//		command has been verified
//		act on the command (cmd)
//
//	for each motor, maintain it's status
////////////////////////////////////////////////////////////////////////////////
void maintain_motors(byte* cmd, bool success){
	// if(stopped){
	// 	continue;
	// }
	uint8_t type 	= cmd_type(cmd);

	if(success){ 	// process the new command
		if(type == CMD_SET_OUTPUTS){	// here we only care if it's set outputs
			uint8_t num_motors_requested;
			FAULT_T retfault;
		}
	}

	// update motor outputs
	uint8_t i 		= 0;
	MotorInfo motor;// = NULL;

	for(i=0; i<DEFAULT_BUF_LEN; i++){
		motor = motor_infos[i];

//		if(motor.hardware == MotorHardware.MH_NONE){
//			continue;
//		}
		// do motor maintainance things
	}
}

////////////////////////////////////////////////////////////////////////////////
// analagous to "execute(cmd)"
// not sure this is necessary
////////////////////////////////////////////////////////////////////////////////
FAULT_T update_motor_infos(byte* cmd){
	return NO_FAULT;
}

////////////////////////////////////////////////////////////////////////////////
// update sensor data requested by client
// not sure this is necessary
////////////////////////////////////////////////////////////////////////////////
FAULT_T update_sensor_infos(byte* cmd){
	return NO_FAULT;
}


////////////////////////////////////////////////////////////////////////////////
//
//  ARDUINO REQUIRED FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////  SETUP
////////////////////////////////////////////////////////////////////////////////
void setup(){
	setup_sensors();
	setup_motors();
	setup_comms();
}

////////////////////////////////////////////////////////////////////////////////
////  MAIN LOOP
////////////////////////////////////////////////////////////////////////////////
void loop(){
	byte cmd[DEFAULT_BUF_LEN];				// to store message from client
	byte rpy[DEFAULT_BUF_LEN]; 				// buffer for the response
	bool success = false;
  FAULT_T fault_code = NO_FAULT;
	if(SerialUSB.available()){
		fault_code = hciRead(cmd);	// verify the command
    Serial.print("CMD STATUS:\t");
    Serial.println(fault_code);
		if(fault_code == NO_FAULT){
			success = true;
		}else{ // there was an issue with the command
			success = false;
			//log_fault(fault_code);			// add to log or whatever
		}
	}
	
	//update_state(cmd); 				// act on the command
	maintain_motors(cmd, success);			// keep robot in a stable state
											// we may not need the cmd argument
	

	if(SerialUSB.available()){
		fault_code = hciAnswer(cmd, rpy);	// reply to the client
    Serial.print("RPY STATUS:\t");
    Serial.println(fault_code);
	}
}
