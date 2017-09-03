package com.cwrubotix.glennifer.hci;

public class LabeledSensorData {
    public LabeledSensorData(int id, SensorData data) {
        this.id = id;
        this.data = data;
    }
    public int id;
    public SensorData data;
}
