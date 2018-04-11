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
	private LinkedList<Coordinate> obstacles; //linked list of coordinates for known obstacles
	private Position position; //contains xy coordinates and heading
	
	public AutonomyState() {
		/*Implementation note: This constructor initiallizes all data members
		 * to 0, including the xy coordinates for the robot position and
		 * the robot's heading. These will not be accurate and should be replaced 
		 * with input as soon as it is availible.
		 */
		
		position = new Position(0.0,0.0,0.0);
		obstacles = new LinkedList();
	}
	
	/* Update methods */
	public void updatePosition (Position position) {
		this.position = position;
	}
	
	public void updatePosition (float x, float y) {
		position.setX(x);
		position.setY(y);
	}
	
	public void updateObstacle (Coordinate obstacle) {
		obstacles.add(obstacle);
	}
	
	/*Getter Methods */
	
	public Position getPosition() {
		return position;
	}
	
	public LinkedList<Coordinate> getObstacles() {
		return obstacles;
	}
}
