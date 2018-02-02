


////////////////////////////////////////////////////////////////////////////////
//	PREPROCESSOR DEFINES
////////////////////////////////////////////////////////////////////////////////

//Command type codes
#define CMD_READ_SENSORS		(0x01)
#define CMD_SET_OUTPUTS			(0x02)
#define CMD_HCI_TEST			(0x03)
#define RPY_HCI_TEST 			(0xA5)

// Sizes & Number of things
#define CMD_HEADER_SIZE			(2)
#define RPY_HEADER_SIZE			(2)
#define HEADER_FIELDS			(3)
#define INSTRUCTION_LEN 		(3)
#define DEFAULT_BUF_LEN 		(256)
#define SENSORE_ID_SIZE 		(1)

// FAULT CODES
#define FAULT_T 				uint16_t
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
	float responsiveness;
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

FAULT_T log_fault(FAULT_T fault){
	if(faultIndex < 255){
		faults[faultIndex++] = fault;
		return NO_FAULT;
	}else{
		faults[faultIndex]   = fault;
		return FAULT_LOG_FULL;
	}
}

void clear_fault_log(){
	FAULT_T tmp[256] = {}; 	// allocate an empty buffer
	faults 			 = tmp;
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

	retval = client.readBytes(cmd, CMD_HEADER_SIZE);	// populate the array

	// proceed to check the cmd array for issues
	if(retval != CMD_HEADER_SIZE){
		return FAULT_INCOMPLETE_HEADER;
	}
	if(!cmd_check_head(cmd)){
		return FAULT_CORRUPTED_HEADER;
	}

	bodyLen = cmd_body_len(cmd);
	retval = client.readBytes(cmd + CMD_HEADER_SIZE, bodyLen);

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
bool cmd_check_head(byte * cmd, uint16_t bodyLen) {
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
FAULT_T hciAnswer(byte* cmd){
	byte rpy[DEFAULT_BUF_LEN];
	bool success = false;
	int size = RPY_HEADER_SIZE;
	uint8_t retval;

	int i = 0;
	FAULT_T fault;
	// while((fault = faults[i]) != 0){
	// 	size += sizeOf(fault);
	// }
	size += faultIndex + 1;	//

	byte rpy[size];
	for(i = 0; i<=faultIndex; i++){
		
	}

	client.write(rpy, size);

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
	retval = client.write(rpy,len+2);
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
	ryp[0] = type;
}
////////////////////////////////////////////////////////////////////////////////
void rpy_set_len(byte* rpy, length){
	rpy[1] = length;
}
////////////////////////////////////////////////////////////////////////////////
uint8_t cmd_sense_sensor_id(cmd, i){
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
//
//	act on the most recent command (cmd)
//		• note: cmd has already been verified
//
////////////////////////////////////////////////////////////////////////////////
void update_state(byte* cmd){
	uint8_t type 	= cmd_type(cmd);
	uint8_t num_sensors_requested;
	uint8_t num_motors_requested;
	FAULT_T retfault;

	rpy_init(rpy, type);

	switch(type){
		case CMD_HCI_TEST:
			rpy_set_len(rpy, RPY_HEADER_SIZE + 1);
			rpy[RPY_HEADER_SIZE] = RPY_HCI_TEST;
			break;

		case CMD_READ_SENSORS:
			num_sensors_requested = cmd_sense_num_sensors(cmd);
			rpy_set_len(rpy, num_sensors_requested);
			for(uint8_t i = 0; i < num_sensors_requested; i++){
				uint8_t id = cmd_sense_sensor_id(cmd, i);
				int16_t val;
				retfault = read_sensor(id, &val);
			}
			break;

		case CMD_SET_OUTPUTS:

			break;
	}
}


////////////////////////////////////////////////////////////////////////////////
//	maintain_motors(byte* cmd)
//		for each motor, update it's status as per cmd
////////////////////////////////////////////////////////////////////////////////
void maintain_motors(byte* cmd){
	byte rpy[DEFAULT_BUF_LEN];	// response buffer
	uint8_t type 	= cmd_type(cmd);

	// update motor outputs
	uint8_t i 		= 0;
	MotorInfo current_motor = NULL;
	while( (current_motor = motor_infos[i]) != NULL){

		// do normal things

		i++;
	
	}
}

////////////////////////////////////////////////////////////////////////////////
// analagous to "execute(cmd)"
////////////////////////////////////////////////////////////////////////////////
FAULT_T update_motor_infos(cmd){
	return NO_FAULT;
}

////////////////////////////////////////////////////////////////////////////////
// update sensor data requested by client
////////////////////////////////////////////////////////////////////////////////
FAULT_T update_sensor_infos(cmd){
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
	if(client.isAvailable()){
		FAULT_T fault_code = hciRead(cmd);	// 
		if(fault_code == NO_FAULT){
			update_state(cmd); 				// act on the cmd
			success = true;
		}else{ // there was an issue with the command
			success = false;
			log_fault(fault_code);		// add to log
		}
	}
	
	maintain_motors(cmd);				// keep robot in a stable state
	

	if(client.isAvailable()){
		fault_code = hciAnswer(cmd);	// reply to the client
	}
}