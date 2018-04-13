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
	MotorInfo* motor 		= &(motor_infos[sensor->whichMotor]); 	// get the motor for this sensor

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

		case SH_BL_ENC_VEL: {
			// motor 		= &(motor_infos[sensor->whichMotor]); 	// get the motor for this sensor
			// MCInfo* board 			= motor->board;
			ODriveArduino* odv1 	= (motor->board->odrive);
			readVal 				= (uint16_t) ( odv1->GetParameter(motor->whichMotor , odv1->PARAM_ENC_VEL)); // returns a float
			sensor->storedVal 		= (sensor->storedVal * (1 - sensor->responsiveness)) + (readVal * sensor->responsiveness);
			*val 					= sensor->storedVal;
			break;}

		case SH_BL_ENC_POS: {
			// motor 		= &(motor_infos[sensor->whichMotor]); 	// get the motor for this sensor
			// MCInfo* board 			= motor->board;
			ODriveArduino* odv2 	= (motor->board->odrive);
			readVal 				= (uint16_t) ( odv2->GetParameter(motor->whichMotor , odv2->PARAM_ENC_POS)); // returns a float
			sensor->storedVal 		= (sensor->storedVal * (1 - sensor->responsiveness)) + (readVal * sensor->responsiveness);
			*val 					= sensor->storedVal;
			break; }
		case SH_LD_CELL: {
			// readVal 				= sensor->loadCell->get_units(3);
			// sensor->storedVal 		= (sensor->storedVal * (1 - sensor->responsiveness)) + (readVal * sensor->responsiveness);
			// *val 					= sensor->storedVal;
			break;}

	}
	return NO_FAULT;
}

int16_t contstrainMag(int16_t og, uint16_t max){
	uint16_t mag 	= abs(og);
	int16_t sign 	= og/mag; // -1 in two's comp := 1111 1111 1111 1111
	int16_t tmp 	= (mag > max ? max : mag); 	// constrain the magnitude
	int16_t retval 	= (int16_t) tmp*sign; 		// correct the sign
	return retval;
}

int16_t ramp(MotorInfo* motor, int16_t newSetPt){
	int16_t delta 	= newSetPt - motor->lastSet;// / delta_t;
	int16_t retval 	= motor->lastSet;
	if(delta > motor->max_delta){
		retval += max_delta;
	}else{
		retval = newSetPt;
	}
	return retval;
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
		// Serial3.print("Command type : ");
		// Serial3.print(type);
		// Serial3.print("  Success Status : ");
		// Serial3.println(success);
		// delay(50);
		// Serial3.println("Time to UPDATE THE MOTORS");
		// delay(50);
		uint8_t num_motors_requested;
		FAULT_T retfault;

		int body_len = cmd_body_len(cmd);

		for(int i = CMD_HEADER_SIZE; i< CMD_HEADER_SIZE+body_len ; i+=3 ){
			uint8_t id 		= cmd[i];
			uint16_t val 	= 0;
			MotorInfo* motor = &(motor_infos[id]); 	// get a pointer to the struct

			val += cmd[i+1];
			val = val << 8;
			val += cmd[i+2];
			Serial3.print("Motor of ID  ");
			Serial3.print(id);
			Serial3.print("\t should be set to ");
			Serial3.println(val);
			motor->setPt = val; 	// deref the ptr and set the struct field
			delay(100);

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
				case MH_ST_POS:
					break;

				case MH_BL_VEL:
					(*(motor->board->odrive)).SetVelocity(motor->whichMotor, motor->setPt);
					break;

				case MH_BL_POS:
					(*(motor->board->odrive)).SetPosition(motor->whichMotor, motor->setPt);
					break;

				case MH_BL_BOTH:
					break;

				case MH_RC_VEL: {
					bool writeSuccess 	= false;
					int16_t newSetPt 	= contstrainMag(motor->setPt, motor->maxDuty);
					if(motor->is_reversed){
						newSetPt = newSetPt * (-1);
					}
					motor->setPt 	= newSetPt;
					newSetPt 		= ramp(motor, newSetPt);
					uint8_t which 	= motor->whichMotor;
					uint8_t address	= (uint8_t) motor->board->addr;
					RoboClaw* rc  	= motor->board->roboclaw;
					Serial3.print("Write to the ROBOCLAW : ");
					Serial3.print(newSetPt);
					Serial3.print("   Duty Cycle : ");
					Serial3.println((newSetPt * 100)/32767);
					delay(100);
					if(which == 0){
						//(*rc).SpeedM1(addr,motor->setPt);
						writeSuccess = roboclawSerial.DutyM1(address,(uint16_t) newSetPt);
					}else{
						//(*rc).SpeedM2(addr,motor->setPt);
						writeSuccess = roboclawSerial.DutyM2(address,(uint16_t) newSetPt);
					}
					motor->lastSet = newSetPt;
					break; }

			}
			motor->lastUpdateTime = millis();
		}
	}


	for(i=0; i<NUM_MOTORS; i++){
		MotorInfo* motor 	= &(motor_infos[i]); 	// get a pointer to the struct
		bool writeSuccess 	= false;

		switch(motor->hardware){
			case MH_NONE:
				break;
			case MH_RC_VEL: {
				int16_t newSetPt 	= motor->lastSet;
				if(motor->setPt != newSetPt){
					newSetPt = ramp(motor, motor->setPt);
					if(motor->which == 0){
						writeSuccess = roboclawSerial.DutyM1(motor->addr,(uint16_t) newSetPt);
					}else{
						writeSuccess = roboclawSerial.DutyM2(motor->addr,(uint16_t) newSetPt);
					}
					motor->lastSet = newSetPt;
					motor->lastUpdateTime = millis();
				}
				break; }
	// 		case MH_ST_PWM:
	// 			Serial3.end();
	// 			Serial3.begin(SABERTOOTH_BAUD);
	// 			digitalWrite( motor->board->selectPin, HIGH);
	// 			(*(motor->board->ST)).motor(motor->whichMotor, motor->setPt);
	// 			delayMicroseconds(50);
	// 			digitalWrite( motor->board->selectPin, LOW);
	// 			Serial3.end();
	// 			Serial3.begin(HCI_BAUD);
	// 			break;

	// 		case MH_ST_VEL:
	// 			// do things
	// 			break;
	// 		case MH_ST_POS:
	// 			break;

	// 		case MH_BL_VEL:
	// 			(*(motor->board->odrive)).SetVelocity(motor->whichMotor, motor->setPt);
	// 			break;

	// 		case MH_BL_POS:
	// 			(*(motor->board->odrive)).SetPosition(motor->whichMotor, motor->setPt);
	// 			break;

	// 		case MH_BL_BOTH:
	// 			break;

		}

		if(writeSuccess){
			motor->lastUpdateTime = millis();
		}
	}
}


#endif
