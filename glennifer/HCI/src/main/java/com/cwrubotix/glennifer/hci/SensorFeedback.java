package com.cwrubotix.glennifer.hci;

public class SensorFeedback {
	int sensorID;
	Actuator.FeedbackType fbType;
	double convFactor;
	double weight;
	long lastInvalidTS;
	int numInvalidTS;
	boolean ignore;
	boolean ignoreD;
	
	SensorFeedback(int ID, Actuator.FeedbackType type, double cf, double w) {
		sensorID = ID;
		fbType = type;
		convFactor = cf;
		weight = w;
	}
	
	public void reset() {
		lastInvalidTS = System.currentTimeMillis();
		numInvalidTS = 0;
		ignore = false;
		ignoreD = false;
	}
}
