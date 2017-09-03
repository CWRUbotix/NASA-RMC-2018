package com.cwrubotix.glennifer.hci;

/**
 * Data for an actuator at a specified time
 */
public class ActuatorData {
	double volt, amp, linvel, linpos,
		    angvel, angpos, torque,
		    force, powerM, powerE,
		    powerH, efficiency, temp;
	long timestamp;
}
