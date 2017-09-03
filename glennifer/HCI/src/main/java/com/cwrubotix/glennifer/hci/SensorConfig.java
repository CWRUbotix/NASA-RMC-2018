package com.cwrubotix.glennifer.hci;

public class SensorConfig {
	String name;
	String description;
	int ID;
	boolean limitSwitch;
	double scale;
        
        //Copy function
        public SensorConfig copy() {
            SensorConfig dum = new SensorConfig();
            dum.name = this.name;
            dum.description = this.description;
            dum.ID = this.ID;
            dum.limitSwitch = this.limitSwitch;
            dum.scale = this.scale;
            return dum;
        }
}
