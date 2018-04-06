package com.cwrubotix.glennifer.robot_state;

import java.time.Instant;

/**
 * A AutonomyState object encapsulates the current state of the robot's 
 * autonomy subsystem. It has update methods to give it data from obstacle
 * detection and localization, and getter methods to query the state. It's
 * update methods can raise fault exceptions for all kinds of reasons. These
 * faults can be responded to using the adjustment method. 
 * 
 * This class does not deal with messages or wire formats. It works purely at
 * the logical level.
 * @author Andrea
 *
 */


public class AutonomyState {

	/* Data members*/
	private LinkedList<Coordinate> obstacles;
	private Position position;
	
	
}
