#ifndef HEADER_FILE
#define HEADER_FILE

////////////////////////////////////////////////////////////////////////////////
void setup_sensors(){
	// SENSORS
	// For testing
	analogReadResolution(ANLG_READ_RES);
	SensorInfo sensor_0;
	sensor_0.hardware 	= SH_PIN_POT;
	sensor_0.whichPin 	= A0;
	sensor_0.responsiveness = 1;
	sensor_0.scale 		= 1;
	sensor_infos[0] 	= sensor_0;

	SensorInfo sensor_1;
	HX711 loadBoi(40,41);
	sensor_1.loadCell = & loadBoi;

}
////////////////////////////////////////////////////////////////////////////////
void setup_motors(){
	analogWriteResolution(ANLG_WRITE_RES);
	// Motor related serial stuff
	Serial1.begin(ODRIVE_BAUD);	// UART with ODrive: 18 (TX) & 19 (RX)
	Serial2.begin(ODRIVE_BAUD);	// UART with ODrive: 16 (TX) & 17 (RX) 
	Serial3.begin(ODRIVE_BAUD); // UART with ODrive: 14 (TX) & 15 (RX)
	
	// Using Simplified Serial mode for the Sabertooths WITH slave select
	// We have to use Serial for them bc software serial isn't supported on the Due
	Serial.begin(SERIAL_BAUD); 			// will be SABERTOOTH_BAUD later
	SabertoothSimplified ST(Serial); 	// Use Serial as the serial port.

	// MOTOR CONTROLLERS (BOARDS)
	// ODrive motors were "setup" in values_and_types.h
	MCInfo odrive_board_0;
	odrive_board_0.odrive = & odrive0;
	board_infos[0] = odrive_board_0;

	MCInfo odrive_board_1;
	odrive_board_1.odrive = & odrive1;
	board_infos[1] = odrive_board_1;

	MCInfo odrive_board_2;
	odrive_board_2.odrive = & odrive2;
	board_infos[1] = odrive_board_2;

	MCInfo board_0;
	board_0.type 		= MC_BRUSHED;
	board_0.ST 			= & ST;
	board_0.selectPin 	= SABERTOOTH_0_SLCT;
	board_infos[4] 		= board_0;

	MCInfo board_1;
	board_1.type 		= MC_BRUSHED;
	board_1.ST 			= & ST;
	board_1.selectPin 	= SABERTOOTH_1_SLCT;
	board_infos[3] 		= board_1;

	pinMode(SABERTOOTH_0_SLCT, OUTPUT);
	pinMode(SABERTOOTH_1_SLCT, OUTPUT);

	// MOTORS
	MotorInfo motor_0;
	motor_0.whichMotor = 0;
	motor_0.board = & odrive_board_0;
	motor_0.hardware = MH_BL_VEL;
	motor_infos[0] = motor_0;

	MotorInfo motor_1;
	motor_1.whichMotor = 1;
	motor_1.board = & odrive_board_0;
	motor_1.hardware = MH_BL_VEL;
	motor_infos[1] = motor_1;

	MotorInfo motor_2;
	motor_2.whichMotor = 0;
	motor_2.board = & odrive_board_1;
	motor_2.hardware = MH_BL_VEL;
	motor_infos[2] = motor_2;

	MotorInfo motor_3;
	motor_3.whichMotor = 1;
	motor_3.board = & odrive_board_1;
	motor_3.hardware = MH_BL_VEL;
	motor_infos[3] = motor_3;

	MotorInfo motor_4;
	motor_4.whichMotor = 0;
	motor_4.board = & odrive_board_2;
	motor_4.hardware = MH_BL_POS;
	motor_infos[4] = motor_4;

	MotorInfo motor_5;
	motor_5.whichMotor = 1;
	motor_5.board = & odrive_board_2;
	motor_5.hardware = MH_BL_POS;
	motor_infos[5] = motor_5;

	MotorInfo motor_6;
	motor_6.whichMotor = 0;
	motor_6.board = & board_0;
	motor_6.hardware = MH_ST_POS;
	motor_infos[6] = motor_6;

	MotorInfo motor_7;
	motor_7.whichMotor = 1;
	motor_7.board = & board_0;
	motor_7.hardware = MH_ST_POS;
	motor_infos[7] = motor_7;

	MotorInfo motor_8;
	motor_8.whichMotor = 0;
	motor_8.board = & board_1;
	motor_8.hardware = MH_ST_POS;
	motor_infos[8] = motor_8;
	

}

#endif

