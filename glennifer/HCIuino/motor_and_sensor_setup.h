#ifndef HEADER_FILE
#define HEADER_FILE

////////////////////////////////////////////////////////////////////////////////
void setup_sensors(){
	// FOREACH SENSOR

}
////////////////////////////////////////////////////////////////////////////////
void setup_motors(){
	// SENSORS
	// For testing
	SensorInfo sensor_0;
	sensor_0.hardware 	= SH_PIN_POT;
	sensor_0.whichPin 	= A0;
	sensor_0.responsiveness = 1;
	sensor_0.scale 		= 1;
	sensor_infos[0] 	= sensor_0;

	// MOTORS
	MotorInfo motor_0;
	motor_0.hardware 	= MH_BR_PWM;
	motor_0.DIRpin  	= 23;
	motor_0.PWMpin 		= 2;
	motor_0.setPt 		= 0;
	motor_0.boardPin 	= BRUSHED_0_SLP;
	motor_infos[0] 		= motor_0;

	MotorInfo motor_1;
	motor_1.hardware 	= MH_BR_PWM;
	motor_1.DIRpin  	= 24;
	motor_1.PWMpin 		= 3;
	motor_1.setPt 		= 0;
	motor_1.boardPin 	= BRUSHED_0_SLP;
	motor_infos[1] 		= motor_1;

	// pinMode(motor_0.PWMpin, OUTPUT);
	// pinMode(motor_1.PWMpin, OUTPUT);
	pinMode(motor_0.DIRpin, OUTPUT);
	pinMode(motor_1.DIRpin, OUTPUT);
	pinMode(BRUSHED_0_SLP, OUTPUT);
	digitalWrite(BRUSHED_0_SLP, HIGH); 
	digitalWrite(motor_0.DIRpin, LOW);
	digitalWrite(motor_1.DIRpin, LOW);
	analogWriteResolution(12);
	// Serial1.begin(ODRIVE_BAUD);	// UART with ODrive - 18 (TX) & 19 (RX)
	// Serial2.begin(ODRIVE_BAUD);	// UART with ODrive - 16 (TX) & 17 (RX) 
	// Serial3.begin(ODRIVE_BAUD); // UART with ODrive - 14 (TX) & 15 (RX)

}


#endif
