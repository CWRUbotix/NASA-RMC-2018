package com.cwrubotix.glennifer.hci;

/**
 * Defines a coordinated actuation between two actuators
 */
public class CoordinatedActuation {
	// If true, overrides all existing actuations that conflict with this one
	boolean override;
	// If true, actuator will hold target once reached
	// If false, actuator will remove this actuation once target is achieved
	boolean hold;
	// ID of the actuator to control
	int actuatorID;
	// ID of the actuator to coordinate to
	int coordinatedActuatorID;
	// The type of actuation, or what data you want to target
	HardwareControlInterface.ActuationType localType;
	// The data to be sourced from the coordinated actuator
	HardwareControlInterface.ActuationType coordType;
	// Map of data from coordinated actuator to target value for the local actuator
	CoordinatedMap map;
	// The current raw output of this actuation
	// Most often will be in PWM
	int currentOutput;
}
