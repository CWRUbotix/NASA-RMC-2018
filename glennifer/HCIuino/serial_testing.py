import serial
import sys

CMD_READ_SENSORS	= 1
CMD_SET_OUTPUTS 	= 2
CMD_HCI_TEST 		= 3
HCI_TEST_SEND 		= 0x5A
HCI_TEST_RPY 		= 0xA5


#===============================================================================
def getHex(dec):
	if dec < 0: 
		dec = 65536+dec
	
	tmp = hex(dec).split('x')[1]
	while len(tmp)<4:
		tmp = '0'+tmp
	
	return tmp

#===============================================================================
def dispAvailableCOM():
	print('='*79)
	print("AVAILABLE COM PORTS:\n")
	#print(serial.tools.list_ports)
	print('='*79)
	print(" ")

#===============================================================================
def dispCMDTypes():
	print('='*79)
	print("CMD tpyes are:")
	print("\t1 : Read Sensor \n\t2 : Set Outputs \n\t3 : HCI Test")
	print("Enter: CMD TYPE \t NUMBER \t ID_1 \t VAL_1 \t ...")
	print('='*79)

#===============================================================================
def readSensors(cmdType, inputs):
	byteArr = [cmdType]
	byteArr.append(0) 			# placeholder until later
	bodyLen 	= 0
	# numSens 	= int(input("How many to read :\t"))
	numSens 	= int(inputs[1])
	respLen 	= 2 + 3*numSens
	bodyLen 	= numSens

	for n in range(0,numSens):
		# ID  = int(input("ID :\t\t\t"))
		ID 	= int(inputs[n + 2])
		byteArr.append(ID)
		

	byteArr[1] = bodyLen
	return byteArr

#===============================================================================
def setOutputs(cmdType, inputs):
	byteArr 	= [cmdType]
	byteArr.append(0) 			# placeholder until later
	respLen 	= 2
	bodyLen 	= 0
	numMtrs = int(inputs[1])
	for n in range(0, numMtrs):
		ID  = int(inputs[2*n + 2])
		val = inputs[2*n + 3]
		print(val)
		# tmp = bytearray.fromhex(val)
		tmp = bytearray.fromhex(getHex( int(val) ))
		byteArr.append(ID)
		byteArr.append(tmp[0])
		byteArr.append(tmp[1])
		bodyLen+=3

	
	# numMtrs 	= int(input("How many to set \t: "))

	# for i in range(0,numMtrs):
	# 	ID  = int(input("ID             \t: "))
	# 	val =     input("Value (in hex) \t: ")
	# 	tmp = bytearray.fromhex(val)
	# 	byteArr.append(ID)
	# 	byteArr.append(tmp[0])
	# 	byteArr.append(tmp[1])
	# 	# byteArr.append(tmp[1])
	#	bodyLen+=3

	byteArr[1] = bodyLen
	return byteArr

#===============================================================================
def main():
	BAUD_RATE 	= 9600
	PORT 		= ''
	try:
		if len(sys.argv[1])>1:
			opt1 = sys.argv[1]
			opt1Arg = sys.argv[2]
			if (opt1[0] == '-') and (len(opt1Arg) > 1):
				if (opt1[1] == 'b' or opt1[1] == 'B'):
					# global BAUD_RATE
					BAUD_RATE = int(opt1[2])
				if (opt1[1] == 'p' or opt1[1] == 'B'):
					PORT = opt1Arg;

	except Exception as e:
		pass


	#dispAvailableCOM()
	
	if(PORT == ''):
		PORT = str(input("Which COM port?:\t"))
	ser 			= serial.Serial()
	ser.port 		= PORT
	ser.baudrate 	= BAUD_RATE

	try:
		ser.open()
		if ser.is_open: print("Serial port is open\n")
		dispCMDTypes()

		done = False
		byteArr = bytearray()
		ser.timeout = 3

		#--------------------------------------------------------------------
		while not done:
			line 	= input("Enter: ")
			inputs 	= line.split(' ')
			cmdType = int(inputs[0]) #int(input("CMD Type:\t"))
			respLen = 0
			byteArr = []

			if cmdType == CMD_HCI_TEST:
				byteArr = [cmdType]
				byteArr.append(1) 
				byteArr.append(HCI_TEST_SEND)
				respLen = 3
				
			elif cmdType == CMD_SET_OUTPUTS:
				byteArr = setOutputs(cmdType, inputs)
				respLen = 2 + byteArr[1];
				
			elif cmdType == CMD_READ_SENSORS:
				byteArr = readSensors(cmdType, inputs)
				respLen = 2 + 3*byteArr[1]
				
			elif cmdType < 0:
				done = True
			else:
				print("Not a recognized command.")


			if not done and len(byteArr)>2: 
				print("Bytes to send:\t"+str(byteArr))
				print("Will expect "+str(respLen)+" bytes.")
				ser.write(byteArr)
				resp = ser.read(respLen)
				printval = "\n === Response ===\n"
				printval +=   "CMD: "
				printval += str(int(resp[0]))
				printval += "\tBODY: "
				printval += str(int(resp[1])) + '\n'
				length = int((respLen-2)/3)
				for i in range(0,length):
					ind = (i*3)+2
					val = int(resp[ind+1])*256
					val += int(resp[ind+2])
					if val > 32767:
						val = 32767-val
					printval += str(int(resp[ind])) + '\t: ' + str(val) + '\n'
				print(printval)

		#--------------------------------------------------------------------
	
	
	except Exception as e:
		print(e)
	
	finally:
		ser.close()


#===============================================================================
if __name__ == '__main__':
	main()