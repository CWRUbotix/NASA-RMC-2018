#ifndef HARDWARE_IO_H_FILE
#define HARDWARE_IO_H_FILE


int16_t low_pass(SensorInfo* sensor, int16_t raw_val){
	// int16_t last 			= sensor->prev_values[0];
	// int16_t lastlast 		= sensor->prev_values[1];
	// int16_t retval 			= (int16_t)( (0.15*raw_val) + (0.3*last) + (0.55*lastlast) );
	int16_t retval = (int16_t)( (0.15*raw_val) + (0.3*sensor->prev_values[0]) + (0.55*sensor->prev_values[1]) );
	sensor->prev_values[1] 	= sensor->prev_values[0];
	sensor->prev_values[0] 	= sensor->storedVal;
	sensor->storedVal 		= retval;
	return retval;
}

int16_t low_pass_aggressive(SensorInfo* sensor, int16_t raw_val, int len){
	int sum = raw_val;
	int multiplier = 1;
	for(int i = 0; i<len; i++){
		multiplier = multiplier << 1; // increase to next power of 2
		sum += multiplier*sensor->prev_values[i]; // first multimplier is 2
		if(i>0){
			sensor->prev_values[i] = sensor->prev_values[i-1];
		}else{
			sensor->prev_values[i] = sensor->storedVal;   }
	}
	int div = (2^(len+2))-1; // equiv. to the sum of all multipliers so far
	return (int16_t)(sum/div);
}

////////////////////////////////////////////////////////////////////////////////
// @param 	limit 	: a pointer to the SensorInfo struct for a limit switch
// @return 	true	: if limit switch is triggered
//  A digital LOW = PRESSED
////////////////////////////////////////////////////////////////////////////////
bool is_limit_triggered(SensorInfo* limit){
	if(limit->hardware != SH_PIN_LIMIT){
		return false;
	}
	bool is_triggered 	= false;

	if(limit->is_reversed){
		// if digitalRead gives HIGH, then we must return TRUE
		is_triggered = digitalRead(limit->whichPin);
	}else{
		// if digitalRead gives LOW, then we must return TRUE
		is_triggered = !digitalRead(limit->whichPin);
	}

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
	bool is_ready = false;

	long start = millis();
	do{
		is_ready = (digitalRead(dat) == LOW);
		if((millis()-start) > HX711_TIMEOUT){
			return -1;
		}
	}while(!is_ready);

	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;
	// shift in the 24-bit number
	data[2] = shiftIn(dat, clk, MSBFIRST);
	data[1] = shiftIn(dat, clk, MSBFIRST);
	data[0] = shiftIn(dat, clk, MSBFIRST);

	// program it with a gain of 128
	digitalWrite(clk, HIGH);
	digitalWrite(clk, LOW);
	
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// build the signed 32-bit integer
	bigData = ( static_cast<unsigned long>(filler) << 24
			| static_cast<unsigned long>(data[2]) << 16
			| static_cast<unsigned long>(data[1]) << 8
			| static_cast<unsigned long>(data[0]) );
	// Serial3.print("LOADY BOI: ");
	// Serial3.println(bigData, DEC);
	// delay(5);

	// float raw_val = (float)((loadyBoi->responsiveness*bigData)-loadyBoi->baseline);
	return (int16_t) ((loadyBoi->responsiveness*bigData)+loadyBoi->baseline);// (raw_val*loadyBoi->responsiveness);
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
int32_t read_enc(MotorInfo* motor){
	int32_t retval = 0;
	SensorInfo* enc = motor->encoder;

	switch(enc->hardware){
		case SH_RC_ENC_VEL:{
			// do nothing
			break;}
		case SH_PIN_POT:{
			int16_t raw = read_pot(enc);
			// int16_t raw = low_pass(enc, read_pot(enc) );	// 
			retval 		= map(raw, enc->val_at_min, enc->val_at_max, motor->minpos, motor->maxpos);
			// Serial3.print("Pot pos: ");
			// Serial3.println(retval);
			// delay(10);
			break;}
		case SH_QUAD_VEL:
			int8_t temp 	= encoder_values[enc->array_index];
			int8_t filler 	= 0;
			if(temp & 0x80){
				filler = 0xFF;
			}else{filler = 0x00;}
			retval = (	((int32_t)filler << 24) | 
						((int32_t)filler << 16) | 
						((int32_t)filler << 8) | 
						temp);

			retval = (enc->is_reversed ? retval*(-1) : retval);
			break;
	}
	return retval;
}

int32_t read_enc(MotorInfo* motor, uint8_t* status, bool* valid){
	return read_enc(motor);
}

int16_t read_current_sensor(SensorInfo* sensor){
	int raw 	= analogRead(sensor->whichPin);
	int ref 	= analogRead(A4); 	// just gotta remember;
	// float volts = (3.3/4095.0) * raw;
	float amps 	= (raw - ref - 1.6)/0.018; 	// i did some math
	// float amps  = (raw*3.3*100.0)/(0.028*4095.0);
	// 0.028 V/A
	// 5V/4095
	// float amps 	= volts/0.028;
	int16_t ret = ((int16_t) amps) - sensor->baseline;
	sensor->storedVal = ret;
	return ret;
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

		case SH_PIN_LIMIT:{
			*val = is_limit_triggered(sensor);
			break;}

		case SH_PIN_POT:{
			int16_t temp = read_pot(sensor);
			if(sensor->whichMotor > 0){
				*val = map(temp, sensor->val_at_min, sensor->val_at_max, motor->minpos, motor->maxpos);
			}else{
				*val = temp;
			}
			
			break;}

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

		case SH_LD_CELL: {
			// *val = read_load_cell(sensor);
			*val = low_pass(sensor, read_load_cell(sensor));
			break;}

		case SH_QUAD_VEL:{
			*val = read_enc(motor);
			break;}
		case SH_BL_CUR:{
			*val 		= read_current_sensor(sensor);
			break;
		}

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

int16_t ramp_up_better(MotorInfo* motor, int16_t newSetPt){
	int16_t delta 	= (newSetPt - motor->lastSet);
	int16_t retval 	= motor->lastSet;

	if((motor->max_delta != 0) && (abs(delta) > motor->max_delta) ){
		retval += (sign(delta) * motor->max_delta);
	}else{
		retval = newSetPt;
	}
	return retval;
}

////////////////////////////////////////////////////////////////////////////////
bool write_to_roboclaw(MotorInfo* motor, int newSetPt){
	return false;
}

////////////////////////////////////////////////////////////////////////////////
bool write_to_sabertooth(MotorInfo* st, int val){
	// we're not gonna use serial for the Sabertooths!!
	// we're using standard servo PWM control
	bool retval 	= false;

	if(st->whichPin != 0){
		val = constrain(val, -(st->max_pwr), st->max_pwr);
		if(st->is_reversed){
			val = val * (-1);
		}
		if(val > 0){
			val = map(val, 0, 500, st->center + st->deadband, 2000);
		}else if( val < 0){
			val = map(val, -500, 0, 1000, st->center - st->deadband);
		}else{
			val = st->center;
		}
		
		//val = map(val, -500, 500, 1000, 2000);
		digitalWrite(st->whichPin, HIGH);
		delayMicroseconds(val - DGTL_WRITE_DELAY);
		digitalWrite(st->whichPin, LOW);

		st->lastUpdateTime = millis();
		retval = true;
	}

	return retval;
}

////////////////////////////////////////////////////////////////////////////////
void reverse_yep(MotorInfo* motor){
	motor->is_reversed = true;
	digitalWrite(motor->board->dir_relay_pin, HIGH);
	delay(RELAY_RISE_FALL_TIME);
}
void unreverse_yep(MotorInfo* motor){
	motor->is_reversed = false;
	digitalWrite(motor->board->dir_relay_pin, LOW);
	delay(RELAY_RISE_FALL_TIME);
}


////////////////////////////////////////////////////////////////////////////////
// @param int16_t val : power value between -1000 & 1000 (inclusive)
////////////////////////////////////////////////////////////////////////////////
bool write_to_yep(MotorInfo* motor, int16_t val){
	ESC* esc 	= motor->board->esc;
	if(val == 0){
		esc->speed(1000);
		return true;
	}

	if( (sign(val) < 0) && !motor->is_reversed){
		reverse_yep(motor);
	}else if( (sign(val) > 0) && motor->is_reversed){
		unreverse_yep(motor);
	}
	
	val 		= abs(constrain(val, -(motor->max_pwr), motor->max_pwr));
	val 		= map(val, 0, motor->max_pwr, motor->center + motor->deadband, motor->center + motor->deadband + motor->max_pwr);
	esc->speed(val);
	motor->lastUpdateTime = millis();
	return true;
}

////////////////////////////////////////////////////////////////////////////////
int get_motor_dir(MotorInfo* motor){
	uint8_t type = motor->hardware;
	if(type==MH_RC_VEL || type==MH_ST_PWM || type==MH_ST_POS || type==MH_BL_VEL){
		return sign(motor->setPt);
	}else if(type==MH_RC_POS || type==MH_ST_POS || type==MH_BL_POS){
		return sign( motor->setPt - motor->lastSet );
	}else{
		return 0;
	}
}

////////////////////////////////////////////////////////////////////////////////
bool is_dir_legal(MotorInfo* motor, SensorInfo* limit){
	int legal_dir 	= limit->mtr_dir_if_triggered;
	//int16_t pos 	= read_enc(motor);
	int16_t dir 	= 0;
	if(motor->hardware == MH_ST_POS){
		dir 		= motor->setPt - read_enc(motor);
	}else{//} if(motor->hardware == MH_ST_PWM || motor->hardware == MH_BL_VEL){
		dir 		= motor->setPt;
	}
	return (sign(dir) == sign(legal_dir));
}

////////////////////////////////////////////////////////////////////////////////
void update_quad_encoders(int8_t* arr){
	int tmp;
	if((tmp=Wire.requestFrom(QUAD_ENC_READER_ADDR, 8)) == 8){
		for(int i = 0; i<8; i++){
			arr[i] = Wire.read();
		}
	} else {
		for(int i = 0; i<8; i++){
			arr[i] = 65+tmp;
		}
		
	}
	// Wire.beginTransmission(QUAD_ENC_READER_ADDR); 	// begin comms w/ the slave device
	// Wire.write(0x01); 								// command to get some data
	// Wire.endTransmission();							// 
	
}

////////////////////////////////////////////////////////////////////////////////
void init_actuators(){
	MotorInfo* port = &(motor_infos[PORT_LIN_ACT_ID]);
	MotorInfo* stbd = &(motor_infos[STARBOARD_LIN_ACT_ID]);
	MotorInfo* tran = &(motor_infos[8]);
	int pos  		= read_enc(port);
	int pos2 		= read_enc(tran);
	port->setPt 	= pos;
	stbd->setPt 	= pos;
	tran->is_stopped = true;
	tran->hardware   = MH_ST_PWM;
	bool temp = write_to_sabertooth(tran, 0);
}
////////////////////////////////////////////////////////////////////////////////
void update_loady_bois(){
	SensorInfo* loadyBoi_1 	= &(sensor_infos[22]);
	SensorInfo* loadyBoi_2 	= &(sensor_infos[23]);
	int16_t temp 			= low_pass(loadyBoi_1, read_load_cell(loadyBoi_1));
	temp 					= low_pass(loadyBoi_2, read_load_cell(loadyBoi_2));
}
////////////////////////////////////////////////////////////////////////////////
void update_current_sensor(){
	int16_t temp = low_pass(&(sensor_infos[33]), read_current_sensor(&(sensor_infos[33])));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//	maintain_motors(byte* cmd)
//	if success is true:
//		command has been verified
//		act on the command (cmd)
//
//	for each motor, maintain it's status
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void maintain_motors(byte* cmd, bool success){

	uint8_t type 	= cmd_type(cmd);
	// so we don't sample the encoder-reader too awfully fast
	if(loops % 2 == 0){
		update_quad_encoders(encoder_values);
		// for(int i = 0; i<8; i++){
		// 	Serial3.print(encoder_values[i]);
		// 	Serial3.print('\t');
		// }
		// Serial3.println();
		// delay(25);
	}

	if(loops % 10 == 0){
		update_loady_bois();
		update_current_sensor();
	}
	
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

			// if 11, we change 8's hardware type to PWM, and pass on the new value
			if(id == 11){
				motor_infos[8].hardware = MH_ST_PWM;
				motor_infos[8].setPt 	= motor->setPt;
			}else if(id == 8){
				// but if 8, set hardware type back to what it was
				motor_infos[8].hardware = MH_ST_POS;
			}

			if(id == 12){
				if(motor->setPt == 1){
					motor_infos[0].hardware = MH_BL_OPEN_LOOP;
					motor_infos[1].hardware = MH_BL_OPEN_LOOP;
					motor_infos[2].hardware = MH_BL_OPEN_LOOP;
					motor_infos[3].hardware = MH_BL_OPEN_LOOP;
				}else{
					motor_infos[0].hardware = MH_BL_VEL;
					motor_infos[1].hardware = MH_BL_VEL;
					motor_infos[2].hardware = MH_BL_VEL;
					motor_infos[3].hardware = MH_BL_VEL;
				}
				
			}
		}
	}

	// actually maintain motors
	// handle any PID, acceleration, faults, etc.
	for(int i=0; i<NUM_MOTORS; i++){
		MotorInfo* motor 	= &(motor_infos[i]); 	// get a pointer to the struct

		uint8_t conflicts 	= 0;
		if(motor->num_limits > 0){
			// check the limit switches, see if the setPt is allowed
			for(int i = 0; i<motor->num_limits; i++){
				SensorInfo* limit 	= motor->limits[i];
				if(is_limit_triggered(limit) && !is_dir_legal(motor, limit)){
					conflicts++;
					motor->is_stopped = true;
				}
			}
		}
		
		if(conflicts == 0){
			motor->is_stopped = false;
		}

		long time 			= millis();
		bool writeSuccess 	= false;
		//bool is_stopped 	= motor->is_stopped;

		switch(motor->hardware){
			case MH_NONE:
				break;
			case MH_ST_PWM:
				if(motor->is_stopped){
					writeSuccess = write_to_sabertooth(motor, 0);
				} else {//if(motor->setPt != motor->lastSet){
					writeSuccess = write_to_sabertooth(motor, motor->setPt);
				}
				break;
			case MH_ST_POS:{
				int32_t pos 	= read_enc(motor);
				int32_t err 	= motor->setPt - pos; // for this, setPt should be a POSITION
				
				if(i == PORT_LIN_ACT_ID){
					MotorInfo* stbd 	= &(motor_infos[STARBOARD_LIN_ACT_ID]);
					int32_t other_pos 	= read_enc(stbd);
					int32_t diff 		= pos - other_pos;
					if( (sign(diff)+sign(err)) == 0 ){//diff > 0){
						motor->kp = LIN_ACT_KP + KP_INC;
						stbd->kp  = LIN_ACT_KP - KP_INC;
					}else {
						motor->kp = LIN_ACT_KP - KP_INC;
						stbd->kp  = LIN_ACT_KP + KP_INC;
					}
				}
				
				int32_t pwr  	= 0;
				if(abs(err) > motor->margin){
					int32_t dt = (time - motor->lastUpdateTime);
					float new_integ = motor->integral + (err * dt);
					motor->integral = ( (fabs(new_integ)*motor->ki ) < 500 ? 
						new_integ : 
						(sign((int)new_integ)*500.0)/(motor->ki)   );

					pwr				= (int32_t) ((motor->kp*err) + (motor->ki*motor->integral));
				}else{
					pwr = 0;
				}
				if(motor->is_stopped){
					pwr = 0;
				}
				
				writeSuccess  	= write_to_sabertooth(motor, pwr);
				
				
				break;}
			case MH_BL_VEL:{
				
				if(motor->is_stopped){
					writeSuccess 		= write_to_yep(motor, 0);
				}else{
					int16_t vel 	= (int16_t) read_enc(motor);
					int16_t err 	= motor->setPt - vel;

					if(motor->setPt == 0){
						motor->current_pwr 	= 0;
					}else if(abs(err) > motor->margin){
						int32_t dt 			= (time - motor->lastUpdateTime);
						float new_integ 	= motor->integral + (err * dt);
						motor->integral 	= 	( (fabs(new_integ)*motor->ki ) < motor->max_pwr ? 
												new_integ : 
												(sign((int)new_integ)*motor->max_pwr)/(motor->ki)   );
						motor->current_pwr 	= (motor->current_pwr + motor->kp*err + motor->ki*motor->integral);
					}


					if(abs(motor->current_pwr) > motor->max_pwr){
						motor->current_pwr = sign(motor->current_pwr) * motor->max_pwr;
					}

					// if client is requesting a direction change
					if(sign(motor->current_pwr) + sign(motor->last_pwr) == 0){
						// set to 0
						writeSuccess 	= write_to_yep(motor, 0); 	// also updates lastUpdateTime
						motor->last_pwr = 0;
					}else if(motor->last_pwr == 0  && (time - motor->lastUpdateTime) >= motor->safe_dt){
						// write to controller (it's now safe to make a direction change)
						writeSuccess 	= write_to_yep(motor, motor->current_pwr);
						motor->last_pwr = motor->current_pwr;
					}else if(sign(motor->last_pwr) == sign(motor->current_pwr) ){ //sign(motor->setPt)){
						// write to controller (no direction change, so no worries)
						writeSuccess 	= write_to_yep(motor, motor->current_pwr); // motor->setPt);
						motor->last_pwr = motor->current_pwr;
					}else{ //} if(motor->setPt == 0){
						writeSuccess 	= write_to_yep(motor, 0);
						motor->last_pwr = 0;
						// DO NOTHING
					}
				}
				break;}
			case MH_LOOKY:
				Herkulex.moveOneAngle(motor->looky_id, constrain(motor->setPt, -159, 159), 200, 2);
				break;
			case MH_BL_OPEN_LOOP:{
				if(motor->is_stopped){
					writeSuccess 		= write_to_yep(motor, 0);
				}else{
					if(sign(motor->setPt) + sign(motor->lastSet) == 0){
						// set to 0
						writeSuccess = write_to_yep(motor, 0); 	// also updates lastUpdateTime
						motor->lastSet = 0;
					}else if(motor->lastSet == 0  && (time - motor->lastUpdateTime) >= motor->safe_dt){
						// write to controller (it's now safe to make a direction change)
						writeSuccess = write_to_yep(motor, motor->setPt);
					}else if(sign(motor->lastSet) == sign(motor->setPt)){
						// write to controller (no direction change, so no worries)
						writeSuccess = write_to_yep(motor, motor->setPt);
					}else if(motor->setPt == 0){
						writeSuccess = write_to_yep(motor, 0);
						motor->lastSet = 0;
						// DO NOTHING
					}
				}
				break;}
			// case MH_ALL:{
			// 	if(motor->setPt > 0){ 		// if steven has set this, time to set up the bc 
			// 		MotorInfo* translate = & motor_infos[8];
			// 		writeSuccess = write_to_sabertooth(translate, 150);
			// 		delay(2000);
			// 		writeSuccess = write_to_sabertooth(translate, 0);
			// 		motor->is_stopped = true; // this is checked to see if we can do position control for bc
			// 		motor->setPt = 0;
			// 	}
			// 	break;}

		}
		// if(writeSuccess){
		// 	motor->lastUpdateTime = millis();
		// }

	}

	
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#endif


