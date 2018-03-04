////////////////////////////////////////////////////////////////////////////////
//
//	PREPROCESSOR INCLUDES
//
////////////////////////////////////////////////////////////////////////////////
#include <ODriveArduino.h>
#include <math.h>

#include "values_and_types.h"
#include "motor_and_sensor_setup.h"
#include "hciRead.h"
#include "hardware_io.h"
#include "hciAnswer.h"


////////////////////////////////////////////////////////////////////////////////
//
//  ARDUINO REQUIRED FUNCTIONS
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////  SETUP
////////////////////////////////////////////////////////////////////////////////
void setup(){
	SerialUSB.begin(HCI_BAUD);	// Begin communication with computer
	Serial.begin(9600); 		// Comms with something-or-other
	setup_sensors();
	setup_motors();
	
	stopped = false;
	Serial.println("================================================================================");
	Serial.println("CMD STATUS | CMD TYPE | BODY LEN | RPY STATUS | RPY SIZE");
}


////////////////////////////////////////////////////////////////////////////////
////  MAIN LOOP
////////////////////////////////////////////////////////////////////////////////
void loop(){
	analogWriteResolution(12);
	byte cmd[DEFAULT_BUF_LEN];				// to store message from client
	byte rpy[DEFAULT_BUF_LEN]; 				// buffer for the response
	bool success = false;
	debugging[0]=0; debugging[1]=0; debugging[2]=0; debugging[3]=0; debugging[4]=0;

	long time = millis() - lastTime;
	FAULT_T fault_code = NO_FAULT;
	if(SerialUSB.available() > 0){
		
		fault_code = hciRead(cmd);	// verify the command

		if(cmd_type(cmd) > 0){
			for(int i = 0; i<cmd_body_len(cmd)+RPY_HEADER_SIZE; i++){
				Serial.print((uint8_t)cmd[i]);
				Serial.print(" ");
			}
			Serial.println();
		}

		debugging[0] = fault_code;

		if(fault_code == NO_FAULT){
			success = true;
		}else{ // there was an issue with the command
			success = false;
			Serial.print("READ ERROR:\t");
			Serial.print(fault_code);
			Serial.print("\t");
			Serial.println(cmd_type(cmd));
			//log_fault(fault_code);			// add to log or whatever
		}
	}else{
		//Serial.println("Client is still not available.");
		if(time >= 5000){
			Serial.println("Client is still not available.");
			lastTime = millis();
		}
		
	}
	maintain_motors(cmd, success);			// keep robot in a stable state
											// we may not need the cmd argument

	if(success){
		Serial.print("SUCCESS! CMD received:\t");
		for(int i = 0; i<cmd_body_len(cmd)+RPY_HEADER_SIZE; i++){
			Serial.print((uint8_t)cmd[i]);
			Serial.print(" ");
		}
		Serial.println();
		fault_code = hciAnswer(cmd, rpy);	// reply to the client

		// Serial.print("\t");
		// Serial.println(fault_code);
		debugging[3] = fault_code;
	}
	// if(debugging[0] != 0){
	// 	Serial.print("   ");
	// 	Serial.print(debugging[0]);
	// 	Serial.print("            ");
	// 	Serial.print(debugging[1]);
	// 	Serial.print("            ");
	// 	Serial.print(debugging[2]);
	// 	Serial.print("      ");
	// 	Serial.print(debugging[3]);
	// 	Serial.print("      ");
	// 	Serial.print(debugging[4]);
	// 	Serial.println();
	// }
}





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

