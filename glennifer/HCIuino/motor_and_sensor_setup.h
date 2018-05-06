#ifndef HEADER_FILE
#define HEADER_FILE

////////////////////////////////////////////////////////////////////////////////
void setup_sensors(){
	// 

	//ENCODERS
	sensor_infos[1].hardware 		= SH_RC_ENC_VEL;
	sensor_infos[1].whichMotor 		= FRONT_PORT_MTR_ID;
	sensor_infos[1].array_index 	= 0;

	sensor_infos[3].hardware 		= SH_RC_ENC_VEL;
	sensor_infos[3].whichMotor 		= FRONT_STARBOARD_MTR_ID;
	sensor_infos[3].array_index 	= 1;

	sensor_infos[5].hardware 		= SH_RC_ENC_VEL;
	sensor_infos[5].whichMotor 		= REAR_STARBOARD_MTR_ID;
	sensor_infos[5].array_index 	= 2;

	sensor_infos[7].hardware 		= SH_RC_ENC_VEL;
	sensor_infos[7].whichMotor 		= REAR_PORT_MTR_ID;
	sensor_infos[7].array_index 	= 3;

	int16_t port_side_low_pass_arr[LOW_PASS_ARRAY_SIZE] = {};
	int16_t stbd_side_low_pass_arr[LOW_PASS_ARRAY_SIZE] = {};
	int16_t translate_low_pass_arr[LOW_PASS_ARRAY_SIZE] = {};

	// port side linear actuator
	sensor_infos[10].hardware 		= SH_PIN_POT;
	sensor_infos[10].val_at_min 	= 85; 		// avg = 84
	sensor_infos[10].val_at_max 	= 4042; 	// avg = 4060
	sensor_infos[10].whichPin 		= A0;
	sensor_infos[10].prev_values 	= port_side_low_pass_arr;
	// starboard side linear actuator
	sensor_infos[11].hardware 		= SH_PIN_POT;
	sensor_infos[11].val_at_min 	= 50; 		// avg = 50
	sensor_infos[11].val_at_max 	= 4027; 	// avg = 4030
	sensor_infos[11].whichPin 		= A1;
	sensor_infos[11].prev_values 	= stbd_side_low_pass_arr;
	// Excavation Translation Linear Pot
	sensor_infos[12].hardware 		= SH_PIN_POT;
	sensor_infos[12].val_at_min 	= 890; 	// not conclusive or definite
	sensor_infos[12].val_at_max 	= 2100; 	// not conclusive or definite
	sensor_infos[12].whichPin 		= A2;
	sensor_infos[12].prev_values 	= translate_low_pass_arr;


	// LIMIT SWITCHES
	// Exc translation, lower
	sensor_infos[13].hardware 		= SH_PIN_LIMIT;
	sensor_infos[13].whichPin 		= 24;
	sensor_infos[13].whichMotor 	= 8;
	sensor_infos[13].mtr_dir_if_triggered = 1; 	
	limit_switches[0] 				= &(sensor_infos[13]);
	// Exc translation; upper, starboard
	sensor_infos[14].hardware 		= SH_PIN_LIMIT;
	sensor_infos[14].whichPin 		= 25;
	sensor_infos[14].whichMotor 	= 8;
	sensor_infos[14].mtr_dir_if_triggered = -1; 
	limit_switches[1] 				= &(sensor_infos[14]);
	// Exc translation; upper, port
	sensor_infos[15].hardware 		= SH_PIN_LIMIT;
	sensor_infos[15].whichPin 		= 26;
	sensor_infos[15].whichMotor 	= 8;
	sensor_infos[15].mtr_dir_if_triggered = -1; 
	limit_switches[2] 				= &(sensor_infos[15]);
	// Exc rotation; port
	sensor_infos[16].hardware 		= SH_PIN_LIMIT;
	sensor_infos[16].whichPin 		= 27;
	sensor_infos[16].whichMotor 	= PORT_LIN_ACT_ID;
	sensor_infos[16].mtr_dir_if_triggered = -1; 
	limit_switches[3] 				= &(sensor_infos[16]);
	// Exc rotation; starboard
	sensor_infos[17].hardware 		= SH_PIN_LIMIT;
	sensor_infos[17].whichPin 		= 28;
	sensor_infos[17].whichMotor 	= STARBOARD_LIN_ACT_ID;
	sensor_infos[17].mtr_dir_if_triggered = -1; 
	limit_switches[4] 				= &(sensor_infos[17]);
	// Dep: upper, starboard
	sensor_infos[18].hardware 		= SH_PIN_LIMIT;
	sensor_infos[18].whichPin 		= 30;
	sensor_infos[18].whichMotor 	= DEP_WINCH_MOTOR_ID;
	sensor_infos[18].mtr_dir_if_triggered = -1; 
	limit_switches[5] 				= &(sensor_infos[18]);
	// Dep: lower, port
	sensor_infos[19].hardware 		= SH_PIN_LIMIT;
	sensor_infos[19].whichPin 		= 31;
	sensor_infos[19].whichMotor 	= DEP_WINCH_MOTOR_ID;
	sensor_infos[19].mtr_dir_if_triggered = 1; 
	limit_switches[6] 				= &(sensor_infos[19]);
	// Dep: lower, starboard
	sensor_infos[20].hardware 		= SH_PIN_LIMIT;
	sensor_infos[20].whichPin 		= 32;
	sensor_infos[20].whichMotor 	= DEP_WINCH_MOTOR_ID;
	sensor_infos[20].mtr_dir_if_triggered = 1; 
	limit_switches[7] 				= &(sensor_infos[20]);
	// Dep: upper, port
	sensor_infos[21].hardware 		= SH_PIN_LIMIT;
	sensor_infos[21].whichPin 		= 33;
	sensor_infos[21].whichMotor 	= DEP_WINCH_MOTOR_ID;
	sensor_infos[21].mtr_dir_if_triggered = -1; 
	limit_switches[8] 				= &(sensor_infos[21]);

	sensor_infos[33].hardware 		= SH_BL_CUR;
	sensor_infos[33].whichPin  		= A3;
}
////////////////////////////////////////////////////////////////////////////////
void setup_motors(){
	// Motor related serial stuff
	Serial2.begin(SABERTOOTH_BAUD);
	SabertoothSimplified ST(Serial2); 	//
	Herkulex.beginSerial2(115200);
	Herkulex.initialize(); //initialize motors
	delay(100);

	board_infos[0].type 			= MC_YEP;
	board_infos[0].esc 				= & ESC_1;
	board_infos[0].dir_relay_pin	= REV_PIN_ESC_1;
	
	board_infos[1].type 			= MC_YEP;
	board_infos[1].esc 				= & ESC_2;
	board_infos[1].dir_relay_pin	= REV_PIN_ESC_2;
	
	board_infos[2].type 			= MC_YEP;
	board_infos[2].esc 				= & ESC_3;
	board_infos[2].dir_relay_pin	= REV_PIN_ESC_3;

	board_infos[5].type 			= MC_YEP;
	board_infos[5].esc 				= & ESC_4;
	board_infos[5].dir_relay_pin	= REV_PIN_ESC_4;

	board_infos[6].type 			= MC_YEP;
	board_infos[6].esc 				= & ESC_5;
	board_infos[6].dir_relay_pin	= REV_PIN_ESC_5;

	board_infos[7].type 			= MC_YEP;
	board_infos[7].esc 				= & ESC_6;
	board_infos[7].dir_relay_pin	= REV_PIN_ESC_6;

	board_infos[3].type 			= MC_SABERTOOTH;
	board_infos[3].ST 				= & ST;
	board_infos[3].selectPin 		= SABERTOOTH_1_SLCT;

	board_infos[4].type 			= MC_SABERTOOTH;
	board_infos[4].ST 				= & ST;
	board_infos[4].selectPin 		= SABERTOOTH_0_SLCT;
	
	pinMode(REV_PIN_ESC_1, OUTPUT);
	pinMode(REV_PIN_ESC_2, OUTPUT);
	pinMode(REV_PIN_ESC_3, OUTPUT);
	pinMode(REV_PIN_ESC_4, OUTPUT);
	pinMode(REV_PIN_ESC_5, OUTPUT);
	pinMode(REV_PIN_ESC_6, OUTPUT);

	pinMode(SABERTOOTH_0_SLCT, OUTPUT);
	pinMode(SABERTOOTH_1_SLCT, OUTPUT);

	digitalWrite(REV_PIN_ESC_1, LOW);
	digitalWrite(REV_PIN_ESC_2, LOW);
	digitalWrite(REV_PIN_ESC_3, LOW);
	digitalWrite(REV_PIN_ESC_4, LOW);
	digitalWrite(REV_PIN_ESC_5, LOW);
	digitalWrite(REV_PIN_ESC_6, LOW);
	

	// MOTORS
	// DRIVE MOTORS
	motor_infos[FRONT_PORT_MTR_ID].hardware 		= MH_BL_VEL;
	motor_infos[FRONT_PORT_MTR_ID].board 			= & (board_infos[1]);
	motor_infos[FRONT_PORT_MTR_ID].max_pwr 			= 1000;
	motor_infos[FRONT_PORT_MTR_ID].center 			= 1000;
	motor_infos[FRONT_PORT_MTR_ID].deadband 		= 250;
	motor_infos[FRONT_PORT_MTR_ID].encoder 			= & (sensor_infos[1]);

	motor_infos[FRONT_STARBOARD_MTR_ID].hardware 	= MH_BL_VEL;
	motor_infos[FRONT_STARBOARD_MTR_ID].board 		= & (board_infos[7]);
	motor_infos[FRONT_STARBOARD_MTR_ID].max_pwr 	= 1000;
	motor_infos[FRONT_STARBOARD_MTR_ID].center 		= 1000;
	motor_infos[FRONT_STARBOARD_MTR_ID].deadband 	= 250;
	motor_infos[FRONT_STARBOARD_MTR_ID].encoder 	= & (sensor_infos[3]);
	
	motor_infos[REAR_PORT_MTR_ID].hardware 			= MH_BL_VEL;
	motor_infos[REAR_PORT_MTR_ID].board 			= & (board_infos[0]);
	motor_infos[REAR_PORT_MTR_ID].max_pwr 			= 1000;
	motor_infos[REAR_PORT_MTR_ID].center 			= 1000;
	motor_infos[REAR_PORT_MTR_ID].deadband 			= 250;
	motor_infos[REAR_PORT_MTR_ID].safe_dt 			= 1000;
	motor_infos[REAR_PORT_MTR_ID].encoder 			= & (sensor_infos[7]);

	motor_infos[REAR_STARBOARD_MTR_ID].hardware 	= MH_BL_VEL;
	motor_infos[REAR_STARBOARD_MTR_ID].board 		= & (board_infos[6]);
	motor_infos[REAR_STARBOARD_MTR_ID].max_pwr 		= 1000;
	motor_infos[REAR_STARBOARD_MTR_ID].center 		= 1000;
	motor_infos[REAR_STARBOARD_MTR_ID].deadband 	= 250;
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
	motor_infos[4].hardware 						= MH_BL_VEL;
	motor_infos[4].max_pwr 							= 1000;
	motor_infos[4].center 							= 1000;
	motor_infos[4].deadband 						= 250;
	motor_infos[4].board 							= & (board_infos[2]);

	// Depostion winch
	motor_infos[5].hardware 						= MH_BL_VEL;
	motor_infos[5].max_pwr 							= 1000;
	motor_infos[5].center 							= 1000;
	motor_infos[5].deadband 						= 250;
	motor_infos[5].board 							= & (board_infos[3]);

	// port-side linear actuator
	motor_infos[PORT_LIN_ACT_ID].whichMotor 		= 0;
	motor_infos[PORT_LIN_ACT_ID].board 				= &(board_infos[3]);
	motor_infos[PORT_LIN_ACT_ID].encoder 			= &(sensor_infos[10]);
	motor_infos[PORT_LIN_ACT_ID].hardware 			= MH_ST_POS;
	motor_infos[PORT_LIN_ACT_ID].whichPin 			= SABERTOOTH_ROT_M2;
	motor_infos[PORT_LIN_ACT_ID].deadband 			= 120;
	motor_infos[PORT_LIN_ACT_ID].is_reversed 		= false;
	motor_infos[PORT_LIN_ACT_ID].margin 			= 10;
	motor_infos[PORT_LIN_ACT_ID].minpos 			= 0;
	motor_infos[PORT_LIN_ACT_ID].maxpos 			= 1000;
	motor_infos[PORT_LIN_ACT_ID].kp 				= LIN_ACT_KP;
	motor_infos[PORT_LIN_ACT_ID].ki 				= LIN_ACT_KI;
	// starboard-side linear actuator
	motor_infos[STARBOARD_LIN_ACT_ID].whichMotor 	= 1;
	motor_infos[STARBOARD_LIN_ACT_ID].board 		= &(board_infos[3]);
	motor_infos[STARBOARD_LIN_ACT_ID].encoder 		= &(sensor_infos[11]);
	motor_infos[STARBOARD_LIN_ACT_ID].hardware 		= MH_ST_POS;
	motor_infos[STARBOARD_LIN_ACT_ID].whichPin 		= SABERTOOTH_ROT_M1;
	motor_infos[STARBOARD_LIN_ACT_ID].deadband 		= 120;
	motor_infos[STARBOARD_LIN_ACT_ID].is_reversed 	= true;
	motor_infos[STARBOARD_LIN_ACT_ID].margin 		= 10;
	motor_infos[STARBOARD_LIN_ACT_ID].minpos 		= 0;
	motor_infos[STARBOARD_LIN_ACT_ID].maxpos 		= 1000;
	motor_infos[STARBOARD_LIN_ACT_ID].kp 			= LIN_ACT_KP;
	motor_infos[STARBOARD_LIN_ACT_ID].ki 			= LIN_ACT_KI;
	
	// translation
	motor_infos[8].whichMotor 						= 0;
	motor_infos[8].board 							= &(board_infos[4]);
	motor_infos[8].encoder 							= &(sensor_infos[12]);
	motor_infos[8].is_reversed 						= false;
	motor_infos[8].whichPin 						= SABERTOOTH_TRANS_M1;
	motor_infos[8].deadband 						= 60;
	motor_infos[8].center 							= 1500;
	motor_infos[8].hardware 						= MH_ST_PWM;
	motor_infos[8].minpos 							= 0;
	motor_infos[8].maxpos 							= 1000;
	motor_infos[8].kp 								= EXC_TRANSLATION_KP;
	motor_infos[8].ki 								= EXC_TRANSLATION_KI;

	motor_infos[9].hardware 						= MH_LOOKY;
	motor_infos[9].looky_id 						= 1;

	motor_infos[10].hardware 						= MH_LOOKY;
	motor_infos[10].looky_id 						= 2;

	pinMode(SABERTOOTH_ROT_M1 ,OUTPUT);
	pinMode(SABERTOOTH_ROT_M2 ,OUTPUT);
	pinMode(SABERTOOTH_TRANS_M1 ,OUTPUT);

}

void init_yeps(){
	ESC_1.arm();                                            // Send the Arm value so the ESC will be ready to take commands
	ESC_2.arm();
	ESC_3.arm();
	ESC_4.arm();
	ESC_5.arm();
	ESC_6.arm();
	delay(5000);                                            // Wait for a while


	ESC_1.speed(1000);
	ESC_2.speed(1000);
	ESC_3.speed(1000);
	ESC_4.speed(1000);
	ESC_5.speed(1000);
	ESC_6.speed(1000);


	delay(1250);
}

#endif
