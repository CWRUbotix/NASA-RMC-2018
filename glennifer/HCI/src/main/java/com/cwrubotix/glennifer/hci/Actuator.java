package com.cwrubotix.glennifer.hci;

import java.util.ArrayList;
import java.util.HashMap;

public class Actuator {
	
	// Type of output
	enum OutputType {
		OutSpeed,
		OutPos;
	}
	
	// Type of feedback
	enum FeedbackType {
		FBAngVel(0),
		FBAngPos(1),
		FBLinVel(2),
		FBLinPos(3),
		FBCurrent(4),
		FBVoltage(5);
		
		// ID
		final int id;
		
		FeedbackType(int i) {
			id = i;
		}
		
		/**
		 * Checks if the current feedback is an integral of the parameter feedback
		 * @param f The feedback type to check against
		 * @return True if it is an integral of f, false if not
		 */
		boolean isIntegralOf(FeedbackType f) {
			if(this.equals(FBAngPos) && f.equals(FBAngVel)) {
				return true;
			}
			if(this.equals(FBLinPos) && f.equals(FBLinVel)) {
				return true;
			}
			return false;
		}
	}
	
	// The configuration for this actuator.  Loaded in constructor
	// Chancing the data in the config after initialization may cause unwanted consequences
	public final ActuatorConfig config;
	
	// Temporary storage for the update() function
	private HashMap<Integer,ArrayList<Double>> tempData = new HashMap<Integer, ArrayList<Double>>();
	// Output type of this actuator
	private OutputType outputType;
	
	/**
	 * Updates the actuator data by polling all of the sensors and fusing the data.
	 * Also checks for faults in sensor data
	 */
	public void update() {
		// Clear the temporary data holder
		for(FeedbackType f: FeedbackType.values()) {
			tempData.get(f.id).clear();
			for(int i = 0; i < 7; i++)
				tempData.get(f.id).add(0.0);
		}
		long now = System.currentTimeMillis();
		// TODO
	}
	
	/**
	 * Creates an actuator given its configuration and controlling interface
	 * @param config The configuration of the actuator
	 * @param iface The interface that controls this actuator
	 */
	Actuator(ActuatorConfig config) {
		// Initialize the temporary data holder
		for(FeedbackType f: FeedbackType.values()) {
			tempData.put(f.id, new ArrayList<Double>());
		}
		this.config = config;
	}
	
}
