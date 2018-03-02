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
//    • takes reply bytes as an array argument 
//    • write these bytes to the client
////////////////////////////////////////////////////////////////////////////////
FAULT_T hciWrite(byte rpy[]){
  uint16_t len  = rpy_len(rpy); // DIFF from last year
  FAULT_T fault = FAULT_FAILED_WRITE;
  uint8_t retval  = SerialUSB.write(rpy,len+2);
  
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
		case CMD_SET_OUTPUTS:
			num = cmd_body_len(cmd)/3;
			uint16_t setPt;
			for(i = 0; i<num; i++){
				ID    = cmd[(3*i)+CMD_HEADER_SIZE];
				setPt = motor_infos[ID].setPt;
				rpy[RPY_HEADER_SIZE+i*3]   = ID;
				rpy[RPY_HEADER_SIZE+i*3+1] = (uint8_t)(setPt >> 8);
				rpy[RPY_HEADER_SIZE+i*3+2] = (uint8_t)(setPt & 0xFF);
				bodyLen+=3;
			}
			break;

	}

	rpy_set_len(rpy, bodyLen); 	// we only want to consider body length
	size 	= RPY_HEADER_SIZE + bodyLen;
	debugging[4] = size;
	retval 	= SerialUSB.write(rpy, size);
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
