package com.cwrubotix.glennifer.hci;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.TimeUnit;

import com.sun.org.apache.xalan.internal.xsltc.trax.TemplatesImpl;
import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortTimeoutException;

public class HardwareControlInterface implements Runnable {
	public static final int baud = 9600;
	public static final byte COMMAND_READ_SENSORS = 0x01;
	public static final byte COMMAND_SET_OUTPUTS = 0x02;
	private static final int SERIAL_TIMEOUT_MS = 2000;
	
	// The types of actuations and constraints that can be made
	enum ActuationType {
		AngVel, AngPos,
		LinVel, LinPos,
		Torque, Force,
		Current, PowerE,
		PowerM, PowerH,
		Temp;
	}

	// Queue of actuations to be checked in
	private LinkedBlockingQueue<Actuation> actuationQueue = new LinkedBlockingQueue<Actuation>();
	// Queue of coordinated actuations to be checked in
	private LinkedBlockingQueue<CoordinatedActuation> coordinatedActuationQueue = new LinkedBlockingQueue<CoordinatedActuation>();
	// Queue of sensor updates detected that can be consumed externally for sending
	private LinkedBlockingQueue<LabeledSensorData> sensorUpdateQueue = new LinkedBlockingQueue<>();
	// List of constraints set on various motors, etc.
	private ArrayList<ActuationConstraint> constraints = new ArrayList<ActuationConstraint>();
	// Hashmap of actuators to their ID's
	private HashMap<Integer, Actuator> actuators = new HashMap<Integer,Actuator>();
	// Hashmap of sensors to their ID's
	private HashMap<Integer, Sensor> sensors = new HashMap<Integer, Sensor>();
	// List of active actuation jobs
	private ArrayList<Actuation> activeActuations = new ArrayList<Actuation>();
	// List of active coordinated actuation jobs
	private ArrayList<CoordinatedActuation> activeCoordinatedActuations = new ArrayList<CoordinatedActuation>();
	private SerialPort port;
	/**
	 * Queue's an actuation to be checked in
	 * @param actuation The actuation job that is to be checked in
	 */
	public void queueActuation(Actuation actuation) {
		actuationQueue.add(actuation);
	}
	
	/**
	 * Queue's a coordinated actuation to be checked in
	 * @param coordinatedActuation The actuation job that is to be checked in
	 */
	public void queueCoordinatedActuation(CoordinatedActuation coordinatedActuation) {
		coordinatedActuationQueue.add(coordinatedActuation);
	}

	/**
	 * Blocking wait until there is a sensor update to be consumed, and then pop it.
	 * @return the sensor data
	 */
	public LabeledSensorData pollSensorUpdate() throws InterruptedException {
		return sensorUpdateQueue.poll(1000000, TimeUnit.DAYS);
	}
	
	/**
	 * Get the sensor object from its ID
	 * @param ID The sensor ID
	 * @return The sensor object
	 */
	public Sensor getSensorFromID(int ID) {
		return sensors.get(ID);
	}
	
	/**
	 * Adds an actuator to the list of actuators
	 * @param actuator The actuator to be added
	 * @param id The ID of the actuator
	 * @return 0 if success, 1 if that ID is already registered
	 */
	public int addActuator(Actuator actuator, int id) {
		if(actuators.containsKey(id)) {
			System.out.println("Fail to add actuator #" + id);
			return 1;
		} else {
			actuators.put(id, actuator);
			return 0;
		}
	}
	
	/**
	 * Adds a sensor to the list of sensors
	 * @param sensor The sensor to be added
	 * @param id The ID of the sensor
	 * @return 0 if success, 1 if that ID is already registered
	 */
	public int addSensor(Sensor sensor, int id) {
		if(sensors.containsKey(id)) {
			return 1;
		} else {
			sensors.put(id, sensor);
			return 0;
		}
	}
	
	@Override
	public void run() {
		while(true) {
			try {
				// Read sensors
				readSensors();
				// Update actuator data
				for(int id:actuators.keySet()) {
					actuators.get(id).update();
				}
				// Process queue of actuations and coordinated actuations
				for(Actuation a:actuationQueue) {
					if(!addActuation(a)) {
						// Send message that actuation was unsuccessful
						System.out.println("Could not add actuation to actuator ID: " + a.actuatorID);
					}
				}
				// Calculate errors in actuation targets with actuator data
				
				// PID
				
				// Calculate errors in coordinated actuation targets with actuator data
				
				// PID
				calcOutputs();
				// Set outputs
				setOutputs();
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					return;
				}
			} catch(SerialPortException | SerialPortTimeoutException e) {
				e.printStackTrace();
				while (true) {
					System.out.println("Trying again in 1 second...");
					try {
						Thread.sleep(1000);
						try {
							port.closePort();
						} catch (SerialPortException e2) {
							System.out.println("Closing port failed");
						}
						String newName = port.getPortName();
						port = new SerialPort(newName);
						port.openPort();
						Thread.sleep(1000);
						port.setParams(baud, 8, 1, 0);
						port.setDTR(false);
						break;
					} catch (InterruptedException e2) {
						return;
					} catch (SerialPortException e2) {
						e.printStackTrace();
					}
				}

			}
		}
	}
	
	/**
	 * Adds an Actuation 
	 * @param act
	 * @return
	 */
	private boolean addActuation(Actuation act) {
		if(!act.type.equals(ActuationType.AngVel) && !act.type.equals(ActuationType.LinVel)) {
			// Only support velocities at this moment
			System.out.println("We only support velocity targets at the moment");
			return false;
		}
		// For all active actuations
		Iterator<Actuation> ita = activeActuations.iterator();
		while(ita.hasNext()) {
			Actuation a = ita.next();
			// If target is already set and override, remove it, otherwise return false
			if(a.actuatorID == act.actuatorID) {
				if(act.override) {
					ita.remove();
				} else {
					return false;
				}
			}
		}
		Iterator<CoordinatedActuation> itca = activeCoordinatedActuations.iterator();
		while(itca.hasNext()) {
			CoordinatedActuation ca = itca.next();
			// For all active actuations
			if(ca.actuatorID == act.actuatorID) {
				// If target is already set and override, remove it, otherwise return false
				if(act.override) {
					itca.remove();
				} else {
					return false;
				}
			}
		}
		// If no conflict or override, add it
		activeActuations.add(act);
		actuationQueue.remove(act);
		return true;
	}
	
	private void calcOutputs() {
		for(int i = 0; i < activeActuations.size(); i++) {
			activeActuations.get(i).currentOutput = (int) (activeActuations.get(i).targetValue/**actuators.get(activeActuations.get(i).actuatorID).config.maxOutput*/);
		}
	}
	
	private boolean setOutputs() throws SerialPortException, SerialPortTimeoutException {
		if(activeActuations.isEmpty()) {
			return true;
		}
		// Allocate byte array for the data in the request
		byte[] data = new byte[activeActuations.size()*4];
		// Generate data array for request
		// Each actuator ID is 2 bytes, each output is 2 bytes
		// Conversion to short is not checked
		for(int i = 0; i < activeActuations.size(); i++) {
			Actuation activeActuation = activeActuations.get(i);
			short actuatorIdShort = (short)activeActuation.actuatorID;
			short currentOutputShort = (short)activeActuation.currentOutput;
			data[4*i] = (byte)(actuatorIdShort >>> 8);
			data[4*i+1] = (byte)(actuatorIdShort);
			data[4*i+2] = (byte)(currentOutputShort >>> 8);
			data[4*i+3] = (byte)(currentOutputShort);
			//System.out.println("Setting output: " + currentOutputShort + " actuator ID: " + actuatorIdShort);
		}
		activeActuations.clear();
		sendMessage(new SerialPacket(COMMAND_SET_OUTPUTS,data));
		// Get the response
		SerialPacket response = readMessage();
		if(response.command != COMMAND_SET_OUTPUTS) {
			System.out.println("Failed to set outputs");
			return false;
		}
		return true;
	}
	
	private boolean readSensors() throws SerialPortException, SerialPortTimeoutException {
		if(sensors.isEmpty()) {
			return true;
		}
		// Get list of sensor IDs
		Integer[] ids = sensors.keySet().toArray(new Integer[sensors.keySet().size()]);
		// Allocate byte array for the data in the request
		byte[] data = new byte[ids.length*2];
		// Generate data array for request
		// Each sensor ID is 2 bytes
		for(int i = 0; i < ids.length; i++) {
			data[2*i] = (byte)(ids[i].intValue()>>8);
			data[2*i+1] = (byte)ids[i].intValue();
		}
		// Send message, prepares it as per the interface
		sendMessage(new SerialPacket(COMMAND_READ_SENSORS,data));
		// Get the response
		SerialPacket response = readMessage();
		long t = System.currentTimeMillis();
		if(response.command != COMMAND_READ_SENSORS) {
			System.out.println("Failed to read sensors");
			return false;
		}
		// Parse the response
		for(int i = 0; i < response.data.length/4; i++) {

			// Parse the sensor IDs
			int sens = ((int)response.data[4*i+0]) << 8 | (0xFF & response.data[4*i+1]);
			// Parse the sensor values
			int dat = ((int)response.data[4*i+2]) << 8 | (0xFF & response.data[4*i+3]);
			if (dat != -32768) {
				// If the sensor is not in the hashmap, ignore it
				if(!sensors.containsKey(sens)) {
					System.out.println("Sensor not loaded (ID = " + sens + ")");
					continue;
				}
				// Get the sensor
				Sensor s = sensors.get(sens);
				// Update it with the data
				boolean different = s.updateRaw(dat);
				if (different) {
					sensorUpdateQueue.add(new LabeledSensorData(sens, new SensorData(dat, t))); // TODO: transform to sensor-specific physical units here
				}
			}
		}
		return true;
	}
	
	private SerialPacket readMessage() throws SerialPortException, SerialPortTimeoutException {
		byte[] r_head = port.readBytes(2, SERIAL_TIMEOUT_MS);
		int len = r_head[1];
		if (len < 0) {
			len += 256;
		}
		byte[] r_body = port.readBytes(len, SERIAL_TIMEOUT_MS);
		SerialPacket response = new SerialPacket(r_head[0],r_body);
		return response;
	}
	
	private void sendMessage(SerialPacket p) throws SerialPortException {
		port.writeBytes(p.asPacket());
	}
	
	public HardwareControlInterface(String spName) {

		// Open the found arduino port
		port = new SerialPort(spName);
		// Try open port
		try {
			port.openPort();
			Thread.sleep(1000);
			port.setParams(baud, 8, 1, 0);
			port.setDTR(false);
		} catch (SerialPortException e) {
			e.printStackTrace();
			return;
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
