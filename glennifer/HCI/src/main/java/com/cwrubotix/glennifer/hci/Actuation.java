package com.cwrubotix.glennifer.hci;

/**
 * Defines the movement of an actuator
 */
public class Actuation {
	// If true, overrides all existing actuations that conflict with this one
	boolean override;
	// If true, actuator will hold target once reached
	// If false, actuator will remove this actuation once target is achieved
	boolean hold;
	// ID of the actuator to control
	int actuatorID;
	// The type of actuation, or what data you want to target
	HardwareControlInterface.ActuationType type;
	// The target value of the actuation
	double targetValue;
	// The current raw output of this actuation
	// Most often will be in PWM
	int currentOutput;
}
