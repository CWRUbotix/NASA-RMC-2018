////////////////////////////////////////////////////////////////////////////////
//	PREPROCESSOR INCLUDES
////////////////////////////////////////////////////////////////////////////////
//#include <ODriveArduino.h>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//	PREPROCESSOR DEFINES
////////////////////////////////////////////////////////////////////////////////

//Command type codes
#define CMD_READ_SENSORS		0x01
#define CMD_SET_OUTPUTS			0x02
#define CMD_HCI_TEST			(0x03)
#define RPY_HCI_TEST 			(0xA5)

// Sizes & Number of things
#define CMD_HEADER_SIZE			(2)
#define RPY_HEADER_SIZE			(2)
#define HEADER_FIELDS			(3)
#define INSTRUCTION_LEN 		(3)
#define DEFAULT_BUF_LEN 		(256)
#define SENSORE_ID_SIZE 		(1)
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
//  SH_RC_POT,
//  SH_RC_ENC,
//  SH_RC_CUR,
  SH_I2C_BAT,
  SH_PIN_LIMIT,
  SH_PIN_POT
};

//SENSOR INFO
typedef struct SensorInfo{
	SensorHardware hardware;
	uint8_t addr; 			// When hardware = SH_RC_*
	uint8_t whichMotor; 	// When hardware = SH_RC_*
	uint8_t whichPin; 		// When hardware = SH_PIN_*
	bool is_reversed;		// When hardware = SH_PIN_LIMIT
	float responsiveness;	// 1 is perfect responsiveness??
	uint16_t scale; 		// 1 unless needed
}SensorInfo;

//MOTOR STUFF
enum MotorHardware {
  MH_NONE,
  MH_RC_PWM,
  MH_RC_VEL,
  MH_RC_POS,
  MH_RC_POS_BOTH,
  MH_ST_PWM,
  MH_ST_POS,
  MH_ST_PWM_BOTH,
  MH_PIN_PWM,
  MH_ALL
};

//MOTOR INFO
typedef struct MotorInfo{
	MotorHardware hardware;
	uint8_t addr;
	uint8_t whichMotor;
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
bool stopped 									= true;



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

}
////////////////////////////////////////////////////////////////////////////////
void setup_motors(){
	
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
	bool success = false;
	uint8_t size = RPY_HEADER_SIZE;
	uint8_t retval;

	FAULT_T fault;
	uint8_t type = cmd_type(cmd);
	rpy_init(rpy, type);
	uint8_t num = 0;
	uint8_t i, ID, index;
	switch(type){
		case CMD_HCI_TEST:
			rpy[RPY_HEADER_SIZE] = RPY_HCI_TEST; 	// data goes immediately after header
			size++;
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
				size += 3;
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
				size+=3;
			}
			break;

	}

	rpy_set_len(rpy, size);
	

	SerialUSB.write(rpy, size);

	if(success){
		clear_fault_log();
		return NO_FAULT;
	}
	return FAULT_FAILED_WRITE;
}

////////////////////////////////////////////////////////////////////////////////
//		• takes reply bytes as an array argument 
//		• write these bytes to the client
////////////////////////////////////////////////////////////////////////////////
FAULT_T hciWrite(byte rpy[]){
	uint8_t retval;
	uint16_t len = rpy_len(rpy);	// DIFF from last year
	retval = SerialUSB.write(rpy,len+2);
	if (retval != len+2) {
    return FAULT_FAILED_WRITE;
  }
  return NO_FAULT;
}


////////////////////////////////////////////////////////////////////////////////
uint16_t rpy_len(byte * rpy){
	uint16_t ret;
	ret += rpy[0];
	ret << 8;
	ret += rpy[1];
	return ret;
}
////////////////////////////////////////////////////////////////////////////////
void rpy_init(byte* rpy, uint8_t type){
	rpy[0] = type;
}
////////////////////////////////////////////////////////////////////////////////
void rpy_set_len(byte* rpy, uint8_t length){
	rpy[1] = length;
}
////////////////////////////////////////////////////////////////////////////////
uint8_t cmd_sense_sensor_id(byte* cmd, uint8_t i){
	uint8_t index = CMD_HEADER_SIZE; 	// index of the first byte of the body
	index += (i * SENSORE_ID_SIZE); 	// index to access the i-th sensor
	return cmd[index]; 					// return the id
}
////////////////////////////////////////////////////////////////////////////////
FAULT_T read_sensor(uint8_t ID, int16_t* val){
	SensorInfo sensor_info = sensor_infos[ID];
	uint8_t status;
	bool valid;
	int32_t val32;
	int16_t dummy;
	int16_t readVal;

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
//
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
	MotorInfo current_motor;// = NULL;
//	while( (current_motor = motor_infos[i]) != NULL){
//
//		// do motor maintaining things
//
//		i++;
//	
//	}
  for(i=0; i<DEFAULT_BUF_LEN; i++){
    current_motor = motor_infos[i];
    // do motor things
  }
}

////////////////////////////////////////////////////////////////////////////////
// analagous to "execute(cmd)"
////////////////////////////////////////////////////////////////////////////////
FAULT_T update_motor_infos(byte* cmd){
	return NO_FAULT;
}

////////////////////////////////////////////////////////////////////////////////
// update sensor data requested by client
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
	}
}





////////////////////////////////////////////////////////////////////////////////
//

//
////////////////////////////////////////////////////////////////////////////////
// void update_state(byte* cmd){
// 	uint8_t type 	= cmd_type(cmd);
// 	uint8_t num_sensors_requested;
// 	uint8_t num_motors_requested;
// 	FAULT_T retfault;

// 	rpy_init(rpy, type);

// 	switch(type){
// 		case CMD_HCI_TEST:
// 			rpy_set_len(rpy, RPY_HEADER_SIZE + 1);
// 			rpy[RPY_HEADER_SIZE] = RPY_HCI_TEST;
// 			break;

// 		case CMD_READ_SENSORS:
// 			num_sensors_requested = cmd_sense_num_sensors(cmd);
// 			rpy_set_len(rpy, num_sensors_requested);
// 			for(uint8_t i = 0; i < num_sensors_requested; i++){
// 				uint8_t id = cmd_sense_sensor_id(cmd, i);
// 				int16_t val;
// 				retfault = read_sensor(id, &val);
// 			}
// 			break;

// 		case CMD_SET_OUTPUTS:

// 			break;
// 	}
// }

