#ifndef HARDWARE_IO_H_FILE
#define HARDWARE_IO_H_FILE


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

		if(motor.hardware == MH_NONE){
			continue;
		}
		// do motor maintainance things
	}
}


#endif