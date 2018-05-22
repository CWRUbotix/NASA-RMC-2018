////////////////////////////////////////////////////////////////////////////////
//
//					HCIuino.ino
// 		2018-05-14 ~1020 Z
//
// 		This is the main source file for the HCI microcontroller board. 
// 		It is designed for use on an Arduino Due.
//
// 		Required Libraries:
// 		○ Sabertooth: 
// 			• https://www.dimensionengineering.com/info/arduino
// 		○ ESC.h
// 			• https://github.com/RB-ENantel/RC_ESC
//
//		This file covers:
//		• setup() Arduino function
// 		• loop() Arduino function
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
//	PREPROCESSOR INCLUDES
//
////////////////////////////////////////////////////////////////////////////////
#include <SabertoothSimplified.h>
#include <ESC.h>
#include <Encoder.h>
#include <Wire.h>
#include <math.h>

#include "Herkulex.h"
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
	Serial.begin(HCI_BAUD); 	// Begin communication with computer
	init_yeps();
	setup_sensors();
	setup_motors(); 
	motor_infos[FRONT_PORT_MTR_ID].setPt = 0;

	Serial3.begin(9600); 		// For the FTDI-USB converter debugging tool
	
	Wire.begin();

	analogWriteResolution(ANLG_WRITE_RES);
	analogReadResolution(ANLG_READ_RES);

	stopped = false;
	init_actuators();
	// Serial3.println("================================================================================");
	// Serial3.println("CMD STATUS | CMD TYPE | BODY LEN | RPY STATUS | RPY SIZE");
	// Serial3.println("================================================================================");
	// delay(50);
}


////////////////////////////////////////////////////////////////////////////////
////  MAIN LOOP
////////////////////////////////////////////////////////////////////////////////
void loop(){
	byte cmd[DEFAULT_BUF_LEN];				// to store message from client
	byte rpy[DEFAULT_BUF_LEN]; 				// buffer for the response
	bool success = false; 

	e_stop_state = digitalRead(E_STOP_PIN);
	if(e_stop_state != e_stop_state_last){
		// we've turned on the e-stop, need to reinitialize the YEPs
		init_yeps();
	}
	e_stop_state_last = e_stop_state;

	long time = millis() - lastTime;
	FAULT_T fault_code = NO_FAULT;
	if(Serial.available() > 0){
		
		fault_code = hciRead(cmd);	// verify the command

		if(cmd_type(cmd) > 0){
			Serial3.print("CMD type:\t");
			Serial3.println(cmd_type(cmd));
			delay(10);
		}

		if(fault_code == NO_FAULT){
			success = true;
		}else{ // there was an issue with the command
			success = false;
			Serial3.print("READ ERROR:\t");
			Serial3.print(fault_code);
			Serial3.print("\t");
			Serial3.println(cmd_type(cmd));
			delay(50);
		}
	}else{
		if(time >= 5000){
			uint32_t avg 	= time / loops;
			lastTime 		= millis();
			loops 			= 0;
			//Serial3.println(avg);
			delay(50);
		}
		
	}
	maintain_motors(cmd, success);			// keep robot in a stable state

	if(success){
		// Serial3.print("CMD:\t");
		// for(int i = 0; i<cmd_body_len(cmd)+RPY_HEADER_SIZE; i++){
		// 	Serial3.print((uint8_t)cmd[i]);
		// 	Serial3.print(" ");
		// }
		// Serial3.println();
		// delay(100);
		fault_code = hciAnswer(cmd, rpy);	// reply to the client
		delay(10);
	}
	loops++;
}