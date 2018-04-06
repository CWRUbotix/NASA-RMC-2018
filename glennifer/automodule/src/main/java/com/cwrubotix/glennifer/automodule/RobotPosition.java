package com.cwrubotix.glennifer.automodule;

public class RobotPosition extends Position{
    
    private float tilt;
    
    
    public RobotPosition(float x_pos, float y_pos, double angle, float tilt){
	super(x_pos, y_pos, angle);
	this.tilt = tilt;
    }
    
    public float getTilt(){
	return tilt;
    }
    
    public void setTilt(float tilt){
	this.tilt = tilt;
    }
}
