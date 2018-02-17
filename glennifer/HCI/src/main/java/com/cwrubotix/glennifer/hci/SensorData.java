package com.cwrubotix.glennifer.hci;

public class SensorData {
	int id;
	double data;
	boolean triggered = false;
	long timestamp;
	SensorData(int id, double data, long timestamp) {
		this.id = id;
		this.data = data;
		this.timestamp = timestamp;
	}
}
