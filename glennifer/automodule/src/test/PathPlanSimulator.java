package test;

import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import java.util.Arrays;
//TODO import GUI stuff


/**
 * 
 * Simulator for different path plan algorithms for autonomy operation
 *
 */
public class PathPlanSimulator{
	
	/** Array of Positions that represent obstacles in the arena*/
	private Position[] obstacles = new Position[6];
	/** Array of Positions that represent obstacles we found*/
	private Position[] obstaclesFound = new Position[6];
	/** The destination of the path*/
	private final Position destination;
	/** 
	 * Array of Positions which represents locations of robots.
	 * <p>We will have one robot per path</p>
	 */
	private Position[] robots;
	/** Array of paths created by different algorithms.*/
	private Path[] paths = new Path[5];
	
	/*Constants:*/
	/** Stores Max straight speed of the robot. Unit: m/s*/
	private final float MAX_STRAIGHT_SPEED = 3.32F;
	/** Stores Max turning speed of the robot. Unit: rad/s*/
	private final float MAX_TURNING_SPEED = (float)Math.PI;
	/** Stores robot's width in meters*/
	private final float ROBOT_WIDTH = 0.75F;
	/** Stores robot's length in meters*/
	private final float ROBOT_LENGTH = 1.0F;
	/** Stores maximum diameter of obstacles in meters*/
	private final float OBSTACLE_SIZE = 0.3F;
	/** Stores the maximum reliable kinect range in meters*/
	private final float KINECT_RANGE = 2.0F;
	/** 
	 * Stores the y direction length of obstacle area of the arena in meters
	 * <p>Actual positions in the arena : y = 1.5m ~ 4.44m</p>
	 */
	private final float OBSTACLE_AREA_HEIGHT = 2.94F;
	/** 
	 * Stores the y direction length of obstacle safe area of the arena in meters
	 * <p>Actual positions in the arena : y = 0.0m ~ 1.5m</p>
	 */
	private final float SAFE_AREA_HEIGHT = 1.5F;
	
	//add more fields/constants if necessary.
	
	/*
	 * IMPORTANT NOTE:
	 * 
	 * For paths array:
	 *   I will just assign indexes to each path created by different algorithms to keep things organized.
	 *   Let me know if you come up with better idea :)
	 *   index 0: path created by midLine algorithm
	 *   index 1: path created by modifiedAStar algorithm.
	 *   index 2: path created by arcPath algorithm
	 *   index 3: path created by Dijkstra algorithm
	 *   index 4 and more will be added as we come up with more algorithms.
	 *   
	 * TODO:
	 *   1) randomObstacleGenerator method - creates 6 obstacles within the arena. DONE
	 *   2) pathGenerator methods - one method per algorithm constructs the field path. Should run on thread and modify path as we see more obstacles.
	 *   3) moveRobot method - should run the thread to display the robot moving with given path and appropriate delays according to robot speed
	 *   4) GUI stuff - displays arena, obstacle, obstacles we see, robot, robot movements. path the robot moved along in real time(On thread)
	 *   5) main method - Runs simulation measure/generate results and either display on GUI or create text file.
	 *   
	 */
	
	/**
	 * Constructor for simulator
	 * <p>
	 * Sets up initial locations of robots, obstacles generated, and destinations along with proper GUI setup.
	 * </p>
	 * @param initialPos the start point of simulation
	 * @param destination the destination of simulation
	 */
	public PathPlanSimulator(Position initialPos, Position destination){
		for(Position robot : robots){
			robot = (Position) initialPos.clone();
		}
		this.destination = destination;
		generateObstacles();
		setUpGUI();
		displayObstacles();
		promptModification();
		displayRobots();
	}
	
	/**
	 * Generates obstacles within the arena.
	 * <p>
	 * Each obstacle is an instance of the position class and fits inside the arena.
	 * <p>
	 * This method ensures the obstacle fits in the arena by taking into account the diameter
	 * and ensuring that the center of the obstacle is in a valid location. 
	 * 
	 * @author Tyler Thieding
	 */
	private void generateObstacles(){
		float validObstacleWidthLength = Position.ARENA_WIDTH() - OBSTACLE_SIZE;
		float validObstacleHeightLength = OBSTACLE_AREA_HEIGHT - OBSTACLE_SIZE;
		for(int i=0; i<obstacles.length; i++) {
			float newObstacleX = (float) (OBSTACLE_SIZE + validObstacleWidthLength*Math.random() - Position.ARENA_WIDTH() / 2);
			float newObstacleY = (float) (OBSTACLE_SIZE + SAFE_AREA_HEIGHT + validObstacleHeightLength*Math.random());
			Position newObstacle = new Position(newObstacleX, newObstacleY, 0, 0);
			obstacles[i] = newObstacle;
		}
	}
	
	/**
	 * Changes position of given obstacle with given coordinates.
	 * @param obstacle the obstacle whose position is being modified
	 * @param x_pos the new x coordinate
	 * @param y_pos the new y coordinate
	 * @return false if given coordinates are invalid as an obstacle.
	 */
	private boolean modifyObstacle(Position obstacle, float x_pos, float y_pos){
		if(y_pos > 1.5F && y_pos < 4.44F){ //if inside obstacle area
			if(!obstacle.setX(x_pos) || !obstacle.setY(y_pos)){
				return false;
			}
			else{
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Returns true if new obstacle has been detected.
	 * @return true if new obstacle has been detected.
	 */
	private boolean foundObstacles(Position robot){
		/*Making sure obstacles are in order of distance from robot*/
		Arrays.sort(obstacles, Position.getComparatorByDistTo(robot));
		Arrays.sort(obstaclesFound, Position.getComparatorByDistTo(robot)); 
		
		int index = 0;
		boolean done = false;
		boolean found = false;
		while(!done){
			while(obstacles[index].equals(obstaclesFound[index]))
				index++;
			if(obstacles[index].getDistTo(robot) < KINECT_RANGE){
				obstaclesFound[index] = obstacles[index];
				found = true;
				markFoundObstacles(obstaclesFound[index]);
			}
			else
				done = true;
		}
		return found;
	}
	
	/*
	 * Creates path by using MidLineAlgorithm
	 * Returns false when failed to create a path.
	 */
	private boolean midLineAlgorithm(){
		//TODO empty method body
		return false;
	}
	
	/*
	 * Creates path by using ModifiedAStar algorithm
	 * Returns false when failed to create a path
	 */
	private boolean modifiedAStar(){
		//TODO empty method body
		return false;
	}
	
	/*
	 * Creates path by using ArcPath algorithm
	 * Returns false when failed to create a path.
	 */
	private boolean arcPathAlgorithm(){
		//TODO empty method body
		return false;
	}
	
	/*
	 * Creates path by using dijkstra algorithm
	 * Returns false when failed to create a path.
	 */
	private boolean dijkstra(){
		//TODO empty method body
		return false;
	}
	
	/**
	 * TODO moves given robot along the given path
	 * @param robot a robot to move
	 * @param path a path to move along
	 * @return true if robot reached destination
	 */
	private boolean moveRobot(Position robot, Path path){
		return false;
	}
	
	/*
	 * Move robots
	 */
	private void moveRobots() throws NoPossiblePathException{
		boolean arrived = false;
		boolean[] robotArrived = new boolean[robots.length];
		Arrays.fill(robotArrived, false);
		while(!arrived){ //Keeps moving till all robots arrive to location
			for(int i = 0; i < robots.length; i++){ //Iterates through each robot
				switch(i){
				case 0: //first robot which is on midLine path
					if(!robotArrived[i] && !moveRobot(robots[i], paths[i])){
						if(foundObstacles(robots[i])){
							if(!midLineAlgorithm())
								throw new NoPossiblePathException(NoPossiblePathException.Cause.MIDLINE);
						}
					}
					else{
						robotArrived[i] = true;
						markArrivedRobot(robots[i]);
					}
					break;
				case 1: //second robot which is on modified Astar path
					if(!robotArrived[i] && !moveRobot(robots[i], paths[i])){
						if(foundObstacles(robots[i])){
							if(!modifiedAStar())
								throw new NoPossiblePathException(NoPossiblePathException.Cause.MODIFIED_ASTAR);
						} 
					}
					else{
						robotArrived[i] = true;
						markArrivedRobot(robots[i]);
					}
					break;
				case 2: //third robot which is on arc path
					if(!robotArrived[i] && !moveRobot(robots[i], paths[i])){
						if(foundObstacles(robots[i])){
							if(!arcPathAlgorithm())
								throw new NoPossiblePathException(NoPossiblePathException.Cause.ARC_PATH);
						}
					}
					else{
						robotArrived[i] = true;
						markArrivedRobot(robots[i]);
					}
					break;
				case 3: //last robot which is on dijkstra path
					if(!robotArrived[i] && !moveRobot(robots[i], paths[i])){
						if(foundObstacles(robots[i])){
							if(!dijkstra())
								throw new NoPossiblePathException(NoPossiblePathException.Cause.DIJKSTRA);
						} 
					}
					else{
						robotArrived[i] = true;
						markArrivedRobot(robots[i]);
					}
					break;
				default: break;
				}
				/*Checks whether all robots arrived at destination*/
				arrived = true;
				for(boolean b : robotArrived){
					if(!b)
						arrived = false;
				}
			}
			displayPaths();
			displayRobots();
		}
	}
	
	/*TODO
	 * GUI METHODS STARTS HERE
	 * 1) setUpGUI()
	 * 2) displayObstacles
	 * 3) promptModification
	 * 4) markFoundObstacles
	 * 5) displayPaths
	 * 6) displayRobots
	 * 7) markFailedPath
	 * 8) markArrivedRobot
	 * 9) displayResult
	 */
	
	private void setUpGUI(){
		//TODO empty method body
	}
	
	private void displayObstacles(){
		//TODO empty method body
	}
	
	private void promptModification(){
		//TODO empty method body
	}
	
	private void markFoundObstacles(Position obstacle){
		//TODO empty method body
	}
	
	private void displayPaths(){
		//TODO empty method body
	}
	
	private void displayRobots(){
		//TODO empty method body
	}
	
	private void markArrivedRobot(Position robot){
		//TODO empty method body
	}
	
	private void markFailedPath(NoPossiblePathException.Cause cause){
		//TODO empty method body
	}
	
	private void displayResult(){
		//TODO empty method body
	}
	
	public void runSimulation(){
		try{
			moveRobots();
		}
		catch(NoPossiblePathException e){
			markFailedPath(e.cause);
			runSimulation();
		}
		displayResult();
	}
	
	//To be done at last
	public static void main(String[] args){
		//TODO empty method body
	}
	
	private static class NoPossiblePathException extends Exception{
		
		private static final long serialVersionUID = 1L;
		private Cause cause;
		private enum Cause{MIDLINE, MODIFIED_ASTAR, ARC_PATH, DIJKSTRA};
		
		public NoPossiblePathException(Cause cause){
			super();
			this.cause = cause;
		}
		
	}
	
}
