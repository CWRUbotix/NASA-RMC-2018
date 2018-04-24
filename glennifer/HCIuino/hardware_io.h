#ifndef HARDWARE_IO_H_FILE
#define HARDWARE_IO_H_FILE

////////////////////////////////////////////////////////////////////////////////
// @param 	limit 	: a pointer to the SensorInfo struct for a limit switch
// @return 	true	: if limit switch is triggered
////////////////////////////////////////////////////////////////////////////////
bool is_limit_triggered(SensorInfo* limit){
	if(limit->hardware != SH_PIN_LIMIT){
		return false;
	}
	uint8_t id 			= 0;
	int16_t tmp 		= 0;
	bool is_triggered 	= false;
	FAULT_T fault 		= NO_FAULT;

	if(limit->is_reversed){
		is_triggered = !digitalRead(limit->whichPin);
	}else{
		is_triggered = digitalRead(limit->whichPin);
	}
	
	is_triggered 			= (bool) (tmp & 1);
	limit->storedVal 		= is_triggered;
	limit->lastUpdateTime 	= millis();

	return is_triggered;
}


////////////////////////////////////////////////////////////////////////////////
// 	@param loadyBio:	ptr to the SensorInfo struct for the sensor
// 	@return uint16_t 	unsigned 16-bit integer. It is a value in grams
////////////////////////////////////////////////////////////////////////////////
uint16_t read_load_cell(SensorInfo* loadyBoi){
	if(loadyBoi->hardware != SH_LD_CELL){
		return 0;
	}
	int clk = loadyBoi->whichPin;
	int dat = loadyBoi->whichPin+1;
	int32_t bigData;

	// shift in the 24-bit number
	for(int i = 0; i<3; i++){
		byte data = shiftIn(dat, clk, MSBFIRST);
		bigData = bigData << 8;
		bigData += data;
	}
	return (loadyBoi->scale * bigData);
}

////////////////////////////////////////////////////////////////////////////////
// Returns a signed number from a potentiometer sensor
// @param pot a pointer to a SensorInfo struct that represents the pot
////////////////////////////////////////////////////////////////////////////////
int16_t read_pot(SensorInfo* pot){
	int16_t readVal = (int16_t)analogRead(pot->whichPin) / pot->scale;
	pot->storedVal 	= (pot->storedVal * (1 - pot->responsiveness)) + (readVal * pot->responsiveness);
	pot->lastUpdateTime = millis();
	return pot->storedVal;
}

////////////////////////////////////////////////////////////////////////////////
// Returns a full width number from the encoder
// Used for PID, etc.
////////////////////////////////////////////////////////////////////////////////
int32_t read_enc(MotorInfo* motor, uint8_t* status, bool* valid){
	int32_t retval = 0;
	SensorInfo* enc = motor->encoder;

	switch(enc->hardware){
		case SH_RC_ENC_VEL:{
			MotorInfo* motor 	= & motor_infos[enc->whichMotor];
			uint8_t address 	= motor->board->addr;
			uint8_t which 		= motor->whichMotor;
			RoboClaw* rc = motor->board->roboclaw;

			
			retval = (which == 0 ? 
				rc->ReadSpeedM1(address, status, valid) : 
				rc->ReadSpeedM2(address, status, valid) );
			// also ReadSpeedMX(addr, ... )

			Serial.print(*valid);
			Serial.print(" ");
		    Serial.print(*status,HEX);
		    Serial.print(" ");
		    Serial.print(retval);
			Serial.print(" ");
			delay(30);
			break;}
		case SH_PIN_POT:{
			int16_t raw = read_pot(enc);
			retval 		= map(raw, enc->val_at_min, enc->val_at_max, motor->minpos, motor->maxpos);
			// Serial3.print("Pot pos: ");
			// Serial3.println(retval);
			delay(10);
			break;}
	}
	return retval;
}

int32_t read_enc(MotorInfo* motor){
	uint8_t status; bool valid;
	return read_enc(motor, &status, &valid);
}

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
			*val = is_limit_triggered(sensor);
			break;

		case SH_PIN_POT:
			*val = read_pot(sensor);
			break;

		case SH_RC_ENC_VEL:{
			uint8_t status;
			bool valid;
			int32_t tmp = read_enc(motor, &status, &valid);
			if(tmp > ((int16_t) 32767) ){
				*val = (int16_t) 32767;
			}else{
				int16_t raw = tmp & 0xFFFF;
				*val = (tmp < 0 ? raw*(-1) : raw);
			}
			break;}
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
			*val 					= read_load_cell(sensor);
			break;}

	}
	return NO_FAULT;
}


////////////////////////////////////////////////////////////////////////////////
// constrains the magnitue of a signed number (og), 
// based on an unsigned number (max)
// @return 	: a signed 16-bit integer with a magnitude <= max
////////////////////////////////////////////////////////////////////////////////
int16_t contstrainMag(int16_t og, uint16_t max){
	uint16_t mag 	= abs(og);
	int16_t sign 	= og/mag; // -1 in two's comp := 1111 1111 1111 1111
	int16_t tmp 	= (mag > max ? max : mag); 	// constrain the magnitude
	int16_t retval 	= (int16_t) tmp*sign; 		// correct the sign
	return retval;
}

////////////////////////////////////////////////////////////////////////////////
// Returns set-point value that does not exceed the motor's maximum delta
// @param motor	: ptr to a MotorInfo struct that represents the target motor
// @param newSetPt : the target/desired set-point to reach
// @return 	: a signed 16-bit int, intended to be the motor's safe set-pt
////////////////////////////////////////////////////////////////////////////////
int16_t ramp_up(MotorInfo* motor, int16_t newSetPt){
	int16_t delta 	= newSetPt - motor->lastSet;// / delta_t;
	int16_t retval 	= motor->lastSet;

	if((motor->max_delta != 0) && (delta > motor->max_delta) ){
		retval += motor->max_delta;
	}else{
		retval = newSetPt;
	}
	
	return retval;
}

////////////////////////////////////////////////////////////////////////////////
bool write_to_roboclaw(MotorInfo* motor, int newSetPt){
	uint8_t address	= (uint8_t) motor->board->addr;
	int16_t sign 	= (motor->is_reversed ? -1 : 1);
	bool retval 	= false;

	if(motor->whichMotor == 0){
		retval = roboclawSerial.DutyM1(address, (uint16_t) (sign*newSetPt) );
	}else{
		retval = roboclawSerial.DutyM2(address, (uint16_t) (sign*newSetPt) );
	}
	if(retval){
		motor->lastUpdateTime = millis();
	}
	return retval;
}

////////////////////////////////////////////////////////////////////////////////
bool write_to_sabertooth(MotorInfo* st, int val){
	// we're not gonna use serial for the Sabertooths!!
	// we're using standard servo PWM control
	bool retval 	= false;

	if(st->whichPin != 0){
		val = constrain(val, -500, 500);
		if(st->is_reversed){
			val = val * (-1);
		}
		val = (val >= 0 ? 
			map(val, 0, 500, 1500 + st->deadband, 2000) : 
			map(val, -500, 0, 1000, 1500 - st->deadband)      );
		
		//val = map(val, -500, 500, 1000, 2000);
		digitalWrite(st->whichPin, HIGH);
		delayMicroseconds(val);
		digitalWrite(st->whichPin, LOW);

		st->lastUpdateTime = millis();
		retval = true;
	}

	return retval;
}

////////////////////////////////////////////////////////////////////////////////
int get_motor_dir(MotorInfo* motor){
	uint8_t type = motor->hardware;
	if(type==MH_RC_VEL || type==MH_ST_PWM || type==MH_ST_POS || MH_BL_VEL){
		return sign(motor->setPt);
	}else if(type==MH_RC_POS || type==MH_ST_POS || type==MH_BL_POS){
		return sign( motor->setPt - motor->lastSet );
	}
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

	uint8_t type 	= cmd_type(cmd);
	
	if(success && (type == CMD_SET_OUTPUTS) ){	// here we only care if it's set outputs
		int body_len = cmd_body_len(cmd);

		for(int i = CMD_HEADER_SIZE; i < CMD_HEADER_SIZE+body_len ; i+=3 ){
			uint8_t id 			= cmd[i];
			uint16_t val 		= 0;
			MotorInfo* motor 	= &(motor_infos[id]); 	// get a pointer to the struct

			val += cmd[i+1];
			val = val << 8;
			val += cmd[i+2];
			motor->lastSet 	= motor->setPt;
			motor->setPt 	= val; 	// deref the ptr and set the struct field

			// make sure the sync'd linear actuators both get updated
			if       (id == PORT_LIN_ACT_ID ){
				(&(motor_infos[STARBOARD_LIN_ACT_ID]))->setPt = val;
			}else if (id == STARBOARD_LIN_ACT_ID){
				(&(motor_infos[PORT_LIN_ACT_ID]))->setPt  	= val;
			}
		}
	}

	// check limit switches & handle collisions
	// for(int i = 0; i < NUM_LIM_SWITCHES; i++){
	// 	SensorInfo* limit = limit_switches[i];

	// 	if(limit->hardware == SH_PIN_LIMIT && is_limit_triggered(limit)){
	// 		MotorInfo* motor = &(motor_infos[limit->whichMotor]);
	// 		int set_pt_sign = get_motor_dir(motor); //sign((int) motor->setPt);
	// 		if(set_pt_sign != limit->mtr_dir_if_triggered){
	// 			// stop the motor!!
	// 			MotorInfo* motor 	= &(motor_infos[limit->whichMotor]);
	// 			motor->is_stopped 	= true;
	// 		}else{
	// 			motor->is_stopped 	= false; 	// make sure we can now move the motor
	// 		}
	// 	}
	// }

	// actually maintain motors
	// handle any PID, acceleration, faults, etc.
	for(int i=0; i<NUM_MOTORS; i++){
		long time 			= millis();
		MotorInfo* motor 	= &(motor_infos[i]); 	// get a pointer to the struct
		bool writeSuccess 	= false;
		bool is_stopped 	= motor->is_stopped;

		switch(motor->hardware){
			case MH_NONE:
				break;
			case MH_ST_PWM:
				if(is_stopped){
					writeSuccess = write_to_sabertooth(motor, 0);
				} else if(motor->setPt != motor->lastSet){
					writeSuccess = write_to_sabertooth(motor, motor->setPt);
				}
				break;
			case MH_ST_POS:{
				if(is_stopped){
					writeSuccess 	= write_to_sabertooth(motor, 0);
				}else{
					int32_t pos 	= read_enc(motor);
					int32_t err 	= motor->setPt - pos; // for this, setPt should be a POSITION
					int32_t pwr  	= 0;
					if(abs(err) > motor->margin){
						float new_integ = motor->integral + (err * (time - motor->lastUpdateTime) );
						motor->integral = ( (fabs(new_integ)*motor->ki ) > 500 ? 
							motor->integral : 
							new_integ );
						pwr				= (int32_t) ((motor->kp*err) + (motor->ki*motor->integral));
					}else{
						pwr = 0;
					}
					Serial3.print(i);
					Serial3.print("\tPOS: ");
					Serial3.print(pos);
					Serial3.print("\tERR: ");
					Serial3.print(err);
					Serial3.print("\tINT: ");
					Serial3.print((int)motor->integral);
					Serial3.print("\tPWR: ");
					Serial3.println(pwr);
					delay(20);
					// pwr should be between -500 and 500
					writeSuccess  	= write_to_sabertooth(motor, pwr);
				}
				break;}
			case MH_RC_VEL: {
				if(is_stopped){
					writeSuccess 		= write_to_roboclaw(motor, 0);
				}else{
					int16_t newSetPt 	= contstrainMag(motor->setPt, motor->maxDuty);
					motor->setPt 		= newSetPt;  // update setPt with the constrained value
					newSetPt 			= ramp_up(motor, newSetPt);
					writeSuccess 		= write_to_roboclaw(motor, newSetPt);
					
					motor->lastSet 		= newSetPt;
				}
				break; }

		}
		// if(writeSuccess){
		// 	motor->lastUpdateTime = millis();
		// }

	}

	
}


#endif

////////////////////////////////////////////////////////////////////////////////
// - checks if set-point is diff from the LAST-set-point (ie: value was updated)
// - constrains the magnitude of the set-point to an acceptable value
// - outputs the motor's set-point to the motor-controller
// 	@param MotorInfo* motor : ptr to a MotorInfo struct for the target motor
////////////////////////////////////////////////////////////////////////////////
// bool update_motor(MotorInfo* motor){
// 	long time 			= millis();
// 	bool writeSuccess 	= false;
// 	switch(motor->hardware){
// 		case MH_NONE:
// 			break;
// 		case MH_ST_PWM:
// 			if(motor->setPt != motor->lastSet){
// 				write_to_sabertooth(motor, motor->setPt);
// 			}
// 			break;

// 		case MH_ST_VEL:
// 			// do things
// 			break;
// 		case MH_ST_POS:{
// 			int32_t pos 	= read_enc(motor);
// 			int32_t err 	= motor->setPt - pos; // for this, setPt should be a POSITION

// 			motor->integral += err * (time - motor->lastUpdateTime);
// 			int32_t pwr 	= (int32_t) ((motor->kp*err) + (motor->ki*motor->integral));
// 			writeSuccess  	= write_to_sabertooth(motor, pwr);
// 			break;}

// 		case MH_BL_VEL:
// 			if(motor->setPt != motor->lastSet){
// 				(*(motor->board->odrive)).SetVelocity(motor->whichMotor, motor->setPt);
// 				writeSuccess = true;
// 			}
// 			break;

// 		case MH_BL_POS:
// 			if(motor->setPt != motor->lastSet){
// 				(*(motor->board->odrive)).SetPosition(motor->whichMotor, motor->setPt);
// 				writeSuccess = true;
// 			}
// 			break;

// 		case MH_BL_BOTH:
// 			break;

// 		case MH_RC_VEL: {
// 			int16_t newSetPt 	= contstrainMag(motor->setPt, motor->maxDuty);
// 			motor->setPt 		= newSetPt;  // update setPt with the constrained value
// 			int16_t sign 		= 1;
// 			if(motor->is_reversed){
// 				sign = (-1);
// 			}
// 			newSetPt 		= ramp_up(motor, newSetPt);
// 			uint8_t which 	= motor->whichMotor;
// 			uint8_t address	= (uint8_t) motor->board->addr;
// 			RoboClaw* rc  	= motor->board->roboclaw;

// 			if(which == 0){
// 				//(*rc).SpeedM1(addr,motor->setPt);
// 				writeSuccess = roboclawSerial.DutyM1(address, (uint16_t) (sign*newSetPt) );
// 			}else{
// 				//(*rc).SpeedM2(addr,motor->setPt);
// 				writeSuccess = roboclawSerial.DutyM2(address, (uint16_t) (sign*newSetPt) );
// 			}
// 			motor->lastSet = newSetPt;
// 			break; }
// 	}
// 	if(writeSuccess){
// 		motor->lastUpdateTime = time;
// 	}
// 	return writeSuccess;
// }

