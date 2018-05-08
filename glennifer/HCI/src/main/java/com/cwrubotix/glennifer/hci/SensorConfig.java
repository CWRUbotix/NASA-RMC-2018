package com.cwrubotix.glennifer.hci;

public class SensorConfig {
	String name;
	String description;
	int ID;
	boolean limitSwitch;
	double scale;
        
    //Copy function
    public SensorConfig copy() {
        SensorConfig dum = new SensorConfig(this.name, this.ID);
        dum.name = this.name;
        dum.description = this.description;
        dum.ID = this.ID;
        dum.limitSwitch = this.limitSwitch;
        dum.scale = this.scale;
        return dum;
    }

    SensorConfig(String name, int ID){
        this.name = name;
        this.scale = 1;
        this.ID = ID;
        this.limitSwitch = false;
    }
}
