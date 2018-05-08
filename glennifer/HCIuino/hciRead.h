#ifndef HCI_READ_H_FILE
#define HCI_READ_H_FILE


////////////////////////////////////////////////////////////////////////////////
bool cmd_check_head(byte * cmd, uint8_t bodyLen) {
  return true;
}
////////////////////////////////////////////////////////////////////////////////
uint16_t cmd_body_len(byte* cmd){
  return cmd[1];  // length is the second byte
}
////////////////////////////////////////////////////////////////////////////////
bool cmd_check_body(byte * cmd, uint8_t bodyLen){
  return (cmd[1] == bodyLen);
}
////////////////////////////////////////////////////////////////////////////////
uint8_t cmd_type(byte cmd[]) {
  return cmd[0];
}
////////////////////////////////////////////////////////////////////////////////
uint8_t cmd_sense_num_sensors(byte* cmd){
  return cmd_body_len(cmd);
}


////////////////////////////////////////////////////////////////////////////////
//		• read & verify data from the client
//		• populate the array argument with this data
////////////////////////////////////////////////////////////////////////////////
FAULT_T hciRead(byte * cmd){
	uint8_t type;
	uint8_t bodyLen;
	uint8_t retval;

	retval = Serial.readBytes(cmd, CMD_HEADER_SIZE);	// populate the array
	if(retval != CMD_HEADER_SIZE){
		return FAULT_INCOMPLETE_HEADER;
	}
	if(!cmd_check_head(cmd, bodyLen)){
		return FAULT_CORRUPTED_HEADER;
	}

	bodyLen = cmd_body_len(cmd);

	
	retval = Serial.readBytes(cmd + CMD_HEADER_SIZE, bodyLen);

	if(retval < bodyLen){
		return FAULT_INCOMPLETE_BODY;
	}
	if (!cmd_check_body(cmd, bodyLen)) {
		return FAULT_CORRUPTED_BODY;
	}

	// success!
	return NO_FAULT;
}



#endif
