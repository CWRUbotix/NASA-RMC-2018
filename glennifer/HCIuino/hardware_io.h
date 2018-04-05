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
		case SH_BL_ENC:
			MotorInfo* motor 		= &(motor_infos[sensor->whichMotor]); 	// get the motor for this sensor
			MCInfo* board 			= motor->board;
			ODriveArduino odv 		= *(board->odrive);
			readVal 				= (uint16_t)odv.GetParameter(motor->whichMotor , odv.PARAM_ENC_VEL); // returns a float
			sensor->storedVal 		= (sensor->storedVal * (1 - sensor->responsiveness)) + (readVal * sensor->responsiveness);
			*val 					= sensor->storedVal;
	}
	return NO_FAULT;
}

uint16_t PID(uint16_t status, MotorInfo* mtr){
	uint32_t now 	= millis();
	uint32_t dT  	= now - (mtr->lastUpdateTime);
	uint16_t error 	= mtr->setPt - status;
	float integral 	= mtr->integral + error*dT;
	float deriv 	= 1.0*(error - mtr->lastError)/dT;

	float retval 	= error*( (mtr->kp) + (mtr->ki)*(integral) + (mtr->kd)*(deriv) ); 
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

	if(success && (type == CMD_SET_OUTPUTS) ){	// here we only care if it's set outputs
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

			// switch(motor->hardware){
			// 	case MH_NONE:
			// 		break;

			// 	case MH_BR_PWM:
			// 		analogWrite(motor->PWMpin,  motor->setPt);
			// 		break;

			// 	case MH_BL_VEL:
			// 		(*(motor->board->odrive)).SetVelocity(motor->whichMotor, motor->setPt);
			// 		break;

			// 	case MH_BL_POS:
			// 		(*(motor->board->odrive)).SetPosition(motor->whichMotor, motor->setPt);
			// 		break;
			// }
			
		}
	}

	// update motor outputs
	uint8_t i 		= 0;
	//MotorInfo motor;// = NULL;

	for(i=0; i<NUM_MOTORS; i++){
		MotorInfo* motor = &(motor_infos[i]); 	// get a pointer to the struct

		switch(motor->hardware){
			case MH_NONE:
				break;

			case MH_ST_PWM:
				digitalWrite( motor->board->selectPin, HIGH);
				(*(motor->board->ST)).motor(motor->whichMotor, motor->setPt);
				delayMicroseconds(50);
				digitalWrite( motor->board->selectPin, LOW);
				break;

			case MH_ST_VEL:
				// do things
				break;

			case MH_BL_VEL:
				(*(motor->board->odrive)).SetVelocity(motor->whichMotor, motor->setPt);
				break;

			case MH_BL_POS:
				(*(motor->board->odrive)).SetPosition(motor->whichMotor, motor->setPt);
				break;
		}

		motor->lastUpdateTime = millis();
	}
}


#endif