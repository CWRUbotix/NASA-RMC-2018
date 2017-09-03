package com.cwrubotix.glennifer.hci;

/**
 * Configuration values for an Actuator
 */
public class ActuatorConfig {
	// Name of the sensor
	public String name;
	// Description of the sensor
	public String description;
	// ID of the actuator
	public int ID;
	// Conversion from linear to angular values
	// linValue*angLin = angValue
	public double angLinConv;
	// True if angular values, false if linear
	public boolean anglin;
	// Slope of the current vs. torque/force graph (current on y axis)
	public double tfCurrentRatio;
	// Stall torque/force (Nm, N)
	public double tfStall;
	// Stall  current (Amps)
	public double stallCurrent;
	// No load velocity (rad/s, m/s)
	public double noLoadVel;
	// No load current (Amps)
	public double noLoadCurrent;
	// Nominal voltage (Volts)
	public double nomVoltage;
	// Maximum int value for output
	public int maxOutput = 127;
	
	public ActuatorConfig copy() {
		ActuatorConfig cop = new ActuatorConfig();
		cop.name = this.name;
		cop.description = this.description;
		cop.ID = this.ID;
		cop.angLinConv = this.angLinConv;
		cop.anglin = this.anglin;
		cop.tfCurrentRatio = this.tfCurrentRatio;
		cop.tfStall = this.tfStall;
		cop.stallCurrent = this.stallCurrent;
		cop.noLoadVel = this.noLoadVel;
		cop.noLoadCurrent = this.noLoadCurrent;
		cop.nomVoltage = this.nomVoltage;
		return cop;
	}
}
