#ifndef HEADER_FILE
#define HEADER_FILE

////////////////////////////////////////////////////////////////////////////////
void setup_sensors(){
	// 

	//ENCODERS
	sensor_infos[1].hardware 		= SH_RC_ENC_VEL;
	sensor_infos[1].whichMotor 		= FRONT_PORT_MTR_ID;

	sensor_infos[3].hardware 		= SH_RC_ENC_VEL;
	sensor_infos[3].whichMotor 		= FRONT_STARBOARD_MTR_ID;

	sensor_infos[5].hardware 		= SH_RC_ENC_VEL;
	sensor_infos[5].whichMotor 		= REAR_STARBOARD_MTR_ID;

	sensor_infos[7].hardware 		= SH_RC_ENC_VEL;
	sensor_infos[7].whichMotor 		= REAR_PORT_MTR_ID;

	// port side linear actuator
	sensor_infos[10].hardware 		= SH_PIN_POT;
	sensor_infos[10].whichPin 		= A0;
	
	// starboard side linear actuator
	sensor_infos[11].hardware 		= SH_PIN_POT;
	sensor_infos[11].whichPin 		= A1;

	// Linear Pot for Excavation Translation
	sensor_infos[12].hardware 		= SH_PIN_POT;
	sensor_infos[12].whichPin 		= A2;

	// LIMIT SWITCHES
	sensor_infos[13].hardware 		= SH_PIN_LIMIT;
	sensor_infos[13].whichPin 		= 24;
	sensor_infos[13].whichMotor 	= 8;
	limit_switches[0] 				= &(sensor_infos[13]);

	sensor_infos[14].hardware 		= SH_PIN_LIMIT;
	sensor_infos[14].whichPin 		= 25;
	sensor_infos[14].whichMotor 	= 8;
	limit_switches[1] 				= &(sensor_infos[14]);

	sensor_infos[15].hardware 		= SH_PIN_LIMIT;
	sensor_infos[15].whichPin 		= 26;
	sensor_infos[15].whichMotor 	= 8;
	limit_switches[2] 				= &(sensor_infos[15]);

	sensor_infos[16].hardware 		= SH_PIN_LIMIT;
	sensor_infos[16].whichPin 		= 27;
	sensor_infos[16].whichMotor 	= 6;
	limit_switches[3] 				= &(sensor_infos[16]);

	sensor_infos[17].hardware 		= SH_PIN_LIMIT;
	sensor_infos[17].whichPin 		= 28;
	sensor_infos[17].whichMotor 	= 7;
	limit_switches[4] 				= &(sensor_infos[17]);

}
////////////////////////////////////////////////////////////////////////////////
void setup_motors(){
	// Motor related serial stuff
	Serial2.begin(SABERTOOTH_BAUD);
	SabertoothSimplified ST(Serial2); 	//
	roboclawSerial.begin(ROBOCLAW_BAUD);

	board_infos[0].type 		= MC_ROBOCLAW;
	board_infos[0].addr 		= ROBOCLAW_0_ADDR;
	board_infos[0].roboclaw 	= & roboclawSerial;
	
	board_infos[1].type 		= MC_ROBOCLAW;
	board_infos[1].addr 		= ROBOCLAW_1_ADDR;
	board_infos[1].roboclaw 	= & roboclawSerial;
	
	board_infos[2].type 		= MC_ROBOCLAW;
	board_infos[2].addr 		= ROBOCLAW_2_ADDR;
	board_infos[2].roboclaw 	= & roboclawSerial;

	board_infos[3].type 		= MC_SABERTOOTH;
	board_infos[3].ST 			= & ST;
	board_infos[3].selectPin 	= SABERTOOTH_1_SLCT;

	board_infos[4].type 		= MC_SABERTOOTH;
	board_infos[4].ST 			= & ST;
	board_infos[4].selectPin 	= SABERTOOTH_0_SLCT;
	
	pinMode(SABERTOOTH_0_SLCT, OUTPUT);
	pinMode(SABERTOOTH_1_SLCT, OUTPUT);
	

	// MOTORS

	// DRIVE MOTORS
	motor_infos[FRONT_PORT_MTR_ID].whichMotor		= 1;
	motor_infos[FRONT_PORT_MTR_ID].hardware 		= MH_RC_VEL;
	motor_infos[FRONT_PORT_MTR_ID].max_delta 		= DFLT_MAX_DELTA;
	motor_infos[FRONT_PORT_MTR_ID].board 			= & (board_infos[2]);
	motor_infos[FRONT_PORT_MTR_ID].encoder 			= & (sensor_infos[1]);

	motor_infos[FRONT_STARBOARD_MTR_ID].whichMotor 	= 1;
	motor_infos[FRONT_STARBOARD_MTR_ID].hardware 	= MH_RC_VEL;
	motor_infos[FRONT_STARBOARD_MTR_ID].is_reversed = true;
	motor_infos[FRONT_STARBOARD_MTR_ID].max_delta 	= DFLT_MAX_DELTA;
	motor_infos[FRONT_STARBOARD_MTR_ID].board 		= & (board_infos[0]);
	motor_infos[FRONT_STARBOARD_MTR_ID].encoder 	= & (sensor_infos[3]);
	
	motor_infos[REAR_PORT_MTR_ID].whichMotor 		= 0;
	motor_infos[REAR_PORT_MTR_ID].hardware 			= MH_RC_VEL;
	motor_infos[REAR_PORT_MTR_ID].max_delta 		= DFLT_MAX_DELTA;
	motor_infos[REAR_PORT_MTR_ID].is_reversed 		= true;
	motor_infos[REAR_PORT_MTR_ID].board 			= & (board_infos[2]);
	motor_infos[REAR_PORT_MTR_ID].encoder 			= & (sensor_infos[7]);

	motor_infos[REAR_STARBOARD_MTR_ID].whichMotor 	= 0;
	motor_infos[REAR_STARBOARD_MTR_ID].hardware 	= MH_RC_VEL;
	motor_infos[REAR_STARBOARD_MTR_ID].max_delta 	= DFLT_MAX_DELTA;
	motor_infos[REAR_STARBOARD_MTR_ID].board 		= & (board_infos[0]);
	motor_infos[REAR_STARBOARD_MTR_ID].encoder 		= & (sensor_infos[5]);

	// Set Roboclaw encoder modes
	roboclawSerial.SetM1EncoderMode(ROBOCLAW_0_ADDR, ENCODER_MODE);
	roboclawSerial.SetM2EncoderMode(ROBOCLAW_0_ADDR, ENCODER_MODE);
	roboclawSerial.SetM1EncoderMode(ROBOCLAW_1_ADDR, ENCODER_MODE);
	roboclawSerial.SetM2EncoderMode(ROBOCLAW_1_ADDR, ENCODER_MODE);
	roboclawSerial.SetM1EncoderMode(ROBOCLAW_2_ADDR, ENCODER_MODE);
	roboclawSerial.SetM2EncoderMode(ROBOCLAW_2_ADDR, ENCODER_MODE);

	// EXCAVATION MOTORS
	// main digging
	motor_infos[4].whichMotor 						= 0;
	motor_infos[4].hardware 						= MH_RC_VEL;
	motor_infos[4].max_delta 						= DFLT_MAX_DELTA;
	motor_infos[4].board 							= & (board_infos[1]);

	// port-side linear actuator
	motor_infos[PORT_LIN_ACT_ID].whichMotor 		= 0;
	motor_infos[PORT_LIN_ACT_ID].board 				= &(board_infos[3]);
	motor_infos[PORT_LIN_ACT_ID].hardware 			= MH_ST_POS;
	motor_infos[PORT_LIN_ACT_ID].minpos 			= 0;
	motor_infos[PORT_LIN_ACT_ID].maxpos 			= 4095;
	motor_infos[PORT_LIN_ACT_ID].kp 				= LIN_ACT_KP;
	motor_infos[PORT_LIN_ACT_ID].ki 				= LIN_ACT_KI;
	// starboard-side linear actuator
	motor_infos[STARBOARD_LIN_ACT_ID].whichMotor 	= 1;
	motor_infos[STARBOARD_LIN_ACT_ID].board 		= &(board_infos[3]);
	motor_infos[STARBOARD_LIN_ACT_ID].hardware 		= MH_ST_POS;
	motor_infos[STARBOARD_LIN_ACT_ID].minpos 		= 0;
	motor_infos[STARBOARD_LIN_ACT_ID].maxpos 		= 4095;
	motor_infos[STARBOARD_LIN_ACT_ID].kp 			= LIN_ACT_KP;
	motor_infos[STARBOARD_LIN_ACT_ID].ki 			= LIN_ACT_KI;
	
	// translation
	motor_infos[8].whichMotor 						= 0;
	motor_infos[8].board 							= &(board_infos[4]);
	motor_infos[8].hardware 						= MH_ST_POS;
	motor_infos[8].minpos 							= 0;
	motor_infos[8].maxpos 							= 4095;
	motor_infos[8].kp 								= EXC_TRANSLATION_KP;
	motor_infos[8].ki 								= EXC_TRANSLATION_KI;

}

#endif
