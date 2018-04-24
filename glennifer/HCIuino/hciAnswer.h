#ifndef HCI_ANSWER_H_FILE
#define HCI_ANSWER_H_FILE


////////////////////////////////////////////////////////////////////////////////
uint8_t rpy_len(byte * rpy){
  // uint16_t ret;
  // ret += rpy[0];
  // ret << 8;
  // ret += rpy[1];
  return rpy[1];
}
////////////////////////////////////////////////////////////////////////////////
void rpy_init(byte* rpy, uint8_t type){
  rpy[0] = type;  // first byte is type
  rpy[1] = 0;   // explicitly initialize to 0
}
////////////////////////////////////////////////////////////////////////////////
void rpy_set_len(byte* rpy, uint8_t length){
  rpy[1] = length;
}
////////////////////////////////////////////////////////////////////////////////
uint8_t cmd_sense_sensor_id(byte* cmd, uint8_t i){
  uint8_t index = CMD_HEADER_SIZE;  // index of the first byte of the body
  index += (i * SENSOR_DATA_SIZE);  // index of the i-th sensor
  return cmd[index];          // return the id
}


////////////////////////////////////////////////////////////////////////////////
// **** NOT USED ****
//    • takes reply bytes as an array argument 
//    • write these bytes to the client
////////////////////////////////////////////////////////////////////////////////
FAULT_T hciWrite(byte rpy[]){
  uint16_t len  = rpy_len(rpy); // DIFF from last year
  FAULT_T fault = FAULT_FAILED_WRITE;
  uint8_t retval  = Serial.write(rpy,len+2);
  
  if (retval == len+2) {
      fault = NO_FAULT;
  }else{
    fault = FAULT_FAILED_WRITE;
  }

  return fault;
}


////////////////////////////////////////////////////////////////////////////////
//		• takes empty byte array as an argument 
//		• populates this with relevant information to respond to the client
////////////////////////////////////////////////////////////////////////////////
FAULT_T hciAnswer(byte* cmd, byte* rpy){
	//byte rpy[DEFAULT_BUF_LEN];
	bool success 	= false;
	uint8_t size 	= RPY_HEADER_SIZE;
	uint8_t bodyLen = 0;
	uint8_t retval 	= 0;
	FAULT_T fault 	= FAULT_FAILED_WRITE; 	// defualt assumption

	uint8_t type = cmd_type(cmd); 			// rpy type is same as cmd type
	rpy_init(rpy, type);
	uint8_t num = 0; 						// number of sensors
	uint8_t i, ID, index; 
	switch(type){
		case CMD_HCI_TEST:
			rpy[RPY_HEADER_SIZE] = RPY_HCI_TEST; 	// data goes immediately after header
			bodyLen++;
			break;
		case CMD_READ_SENSORS:
			int16_t value;
			num = cmd_sense_num_sensors(cmd);
			for(i = 0; i<num; i++){
				ID    = cmd[i+CMD_HEADER_SIZE];
				read_sensor(ID, &value);
				rpy[(3*i)+RPY_HEADER_SIZE]   = ID;
				rpy[(3*i)+RPY_HEADER_SIZE+1] = (uint8_t)(value >> 8);
				rpy[(3*i)+RPY_HEADER_SIZE+2] = (uint8_t)(value & 0xFF);
				bodyLen += 3;
			}
			break;
		case CMD_SET_OUTPUTS:{
			num = cmd_body_len(cmd)/3;
			uint16_t setPt;
			uint8_t cmd_index = CMD_HEADER_SIZE;
			uint8_t rpy_index = RPY_HEADER_SIZE;
			for(i = 0; i<num; i++){
				cmd_index += (3*i);
				rpy_index += (3*i);
				ID    = cmd[cmd_index];
				setPt = motor_infos[ID].setPt;
				rpy[rpy_index]   = ID;
				rpy[rpy_index+1] = (uint8_t)(setPt >> 8);
				rpy[rpy_index+2] = (uint8_t)(setPt & 0xFF);
				bodyLen+=3;
			}
			break;}
		case CMD_SET_PID:{
			uint8_t cmd_index = CMD_HEADER_SIZE;
			uint8_t rpy_index = RPY_HEADER_SIZE;
			num = cmd_body_len(cmd)/3;
			for(i = 0; i < num; i++){
				cmd_index 		 += (3*i);
				rpy_index 		 += (3*i);
				ID 				 = cmd[cmd_index];
				MotorInfo* motor = &(motor_infos[ID]);
				uint8_t member 	 = cmd[cmd_index+1];
				int8_t scale	 = cmd[cmd_index+2];

				switch (member){
					case 0:
						motor->kp += (1.0 * scale * KP_INC);
						break;
					case 1:
						motor->ki += (1.0 * scale * KI_INC);
						break;
				}

			}
			break;}
	}

	rpy_set_len(rpy, bodyLen); 	// we only want to consider body length
	size 	= RPY_HEADER_SIZE + bodyLen;
	debugging[4] = size;

	retval 	= Serial.write(rpy, size); 	// Actually write to the client
	
	success = (retval == size);
	if(success){
		//clear_fault_log();
		fault = NO_FAULT;
	}else{
		fault = FAULT_FAILED_WRITE;
	}

	return fault;
}

#endif
