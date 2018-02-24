#ifndef HEADER_FILE
#define HEADER_FILE

////////////////////////////////////////////////////////////////////////////////
void setup_sensors(){
	// FOREACH SENSOR

}
////////////////////////////////////////////////////////////////////////////////
void setup_motors(){
	// FOREACH MOTOR
	Serial1.begin(ODRIVE_BAUD);	// UART with ODrive - 18 (TX) & 19 (RX)
	Serial2.begin(ODRIVE_BAUD);	// UART with ODrive - 16 (TX) & 17 (RX) 
	Serial3.begin(ODRIVE_BAUD); // UART with ODrive - 14 (TX) & 15 (RX)
}


#endif
