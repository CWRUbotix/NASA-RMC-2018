#ifndef HARDWARE_IO_H_FILE
#define HARDWARE_IO_H_FILE


////////////////////////////////////////////////////////////////////////////////
// 	@param ID:	takes a sensor ID
// 	@param val:	ptr to the 16-bit signed int where the sensor data will go
////////////////////////////////////////////////////////////////////////////////
FAULT_T read_sensor(uint8_t ID, int16_t* val){
	SensorInfo* sensor 		= &(sensor_infos[ID]);
	uint8_t status 			= 0;
	bool valid 				= false;
	int32_t val32 			= 0;
	int16_t dummy 			= 0;
	int16_t readVal 		= 0;

	switch (sensor->hardware) {
		case SH_PIN_LIMIT:
			if(sensor->is_reversed){
				*val = !digitalRead(sensor->whichPin);
			}else{
				*val = digitalRead(sensor->whichPin);
			}
			break;

		case SH_PIN_POT:
			readVal 				= (int16_t)analogRead(sensor->whichPin) / sensor->scale;
			sensor->storedVal 		= (sensor->storedVal * (1 - sensor->responsiveness)) + (readVal * sensor->responsiveness);
			*val 					= sensor->storedVal;
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

			for(int i = CMD_HEADER_SIZE; i< CMD_HEADER_SIZE+cmd_body_len(cmd) ; i+=3 ){
				uint8_t id 		= cmd[i];
				uint16_t val 	= 0;
				MotorInfo* motor = &(motor_infos[id]); 	// get a pointer to the struct


				val += cmd[i+1];
				val = val << 8;
				val += cmd[i+2];
				Serial.print("Value Received:\t");
				Serial.println(val);
				motor->setPt = val; 	// deref the ptr and set the struct field

				if(motor->hardware == MH_NONE){
					continue;
				}else if(motor->hardware == MH_BR_PWM){
					//Serial.print("Set Point for this motor: ");
					//Serial.println(val);
					Serial.print("Writing ");
					Serial.print(motor->setPt);
					Serial.print(" to the motor - on pin ");
					Serial.println(motor->PWMpin);
					analogWrite(motor->PWMpin,  motor->setPt);
				}
			}
		}
	}

	// update motor outputs
	uint8_t i 		= 0;
	//MotorInfo motor;// = NULL;

	for(i=0; i<NUM_MOTORS; i++){
		MotorInfo* motor = &(motor_infos[i]); 	// get a pointer to the struct

		if(motor->hardware == MH_NONE){
			continue;
		}else {
			//val = 
		}

		if(motor->hardware == MH_BR_PWM){
			Serial.print("Maintaining Motor ");
			Serial.print(i);
			Serial.print(".\tSET PT:\t");
			Serial.println(motor->setPt);
			// do PID
			analogWrite(motor->PWMpin , motor->setPt );
		}

		// do motor maintainance things
	}
}


#endif