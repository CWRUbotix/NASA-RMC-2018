package com.cwrubotix.glennifer.hci;

public class SensorData {
	double data;
	boolean triggered = false;
	long timestamp;
	SensorData(double data, long timestamp) {
		this.data = data;
		this.timestamp = timestamp;
	}
}
