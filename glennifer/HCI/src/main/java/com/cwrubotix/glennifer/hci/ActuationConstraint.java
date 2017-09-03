package com.cwrubotix.glennifer.hci;

import com.cwrubotix.glennifer.hci.HardwareControlInterface.ActuationType;

/**
 * Constrains an actuator
 */
public class ActuationConstraint {
	// The type of actuation, or data, to constrain
	ActuationType type;
	// If true, the value is a maximum
	// If false, the value is a minimum
	boolean minMax;
	// The value of the constraint
	double value;
}
