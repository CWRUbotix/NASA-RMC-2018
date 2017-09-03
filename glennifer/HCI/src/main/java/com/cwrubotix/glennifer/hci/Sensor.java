package com.cwrubotix.glennifer.hci;

import java.util.ArrayList;

public class Sensor {
	// Number of milliseconds that data is valid for
	final long maxExtrapolate = 250;
	
	public final SensorConfig config;
	
	ArrayList<SensorData> data = new ArrayList<SensorData>();
	double RC;
	boolean lowPass = false;
	
	public boolean updateRaw(int val) {
		return update(val*config.scale);
	}
	
	/**
	 * Updates the data with the value.  Performs low pass filter if enabled. Returns whether or not the new value is different
	 * @param val
	 * @return true if the new value is different, false if not
	 */
	public boolean update(double val) {
		long mil = System.currentTimeMillis();
		if(data.size() == 0 || !lowPass) {
			data.add(new SensorData(val, mil));
			return true;
		}
		double dt = (double)(mil - data.get(data.size()-1).timestamp);
		double alpha = dt/(RC + dt);
		double fVal = alpha * val + (1 - alpha)*data.get(data.size()-1).data;
		boolean different = (fVal != data.get(data.size() - 1).data);
		data.add(new SensorData(fVal, mil));
		return different;
	}
	
	public void setLowPassFreq(double freq) {
		RC = 1/(2*Math.PI*freq);
		lowPass = true;
	}
	
	public void removeLowPass() {
		lowPass = false;
	}
	
	public double getDataAt(long millis) {
		if(data.size() == 0) {
			return 0;
		}
		if(data.size() == 1) {
			return data.get(0).data;
		}
		long lastTimestamp = data.get(data.size() - 1).timestamp;
		long firstTimestamp = data.get(0).timestamp;
		if(millis - lastTimestamp > maxExtrapolate) {
			return data.get(data.size() - 1).data;
		}
		if(millis < firstTimestamp) {
			return 0;
		}
		int b = 0, a = 0;
		for(int i = 0; i < data.size(); i++) {
			if(data.get(i).timestamp == millis) {
				return millis;
			}
			if(data.get(i).timestamp < millis) {
				b = i;
			} else if(data.get(i).timestamp > millis) {
				a = i;
				break;
			}
		}
		SensorData after = data.get(a);
		SensorData before = data.get(b);
		double slope = (after.data - before.data)/(after.timestamp - before.timestamp);
		return before.data + slope*(millis - before.timestamp);
	}
	
	public double getDerivativeAt(long millis) {
		if(data.size() <= 1) {
			return 0;
		}
		// Todo
		return 0;
	}
	
	Sensor(SensorConfig config) {
		this.config = config;
	}
}
