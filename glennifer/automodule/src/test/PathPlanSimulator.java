package test;

import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import java.lang.Thread;
import java.lang.Runnable;
//TODO import GUI stuff


/**
 * 
 * Simulator for different path plan algorithms for autonomy operation
 *
 */
public class PathPlanSimulator{
	private Position[] obstacles = new Position[6];
	private Position[] obstaclesFound = new Position[6];
	private Position destination;
	private Position currentPos;
	private Thread[] threads;
	private Path[] paths = new Path[5]; //I will create Path data type unless someone really wants to...?
	
	/*Constants:*/
	private final float STRAIGHT_SPEED = 0.0F;  //Will be filled when
	private final float TURNING_SPEED = 0.0F;   //Locomotion gives us numbers.
	private final float ROBOT_WIDTH = 0.75F;  // Will be modified whenever
	private final float ROBOT_LENGTH = 1.0F;  // Estimate on robot size changes
	private final float OBSTACLE_SIZE = 0.3F; //In diameter, Assume all obstacles are max-sized
	private final float KINECT_RANGE = 2.0F; //How far we can see. Will be tested and adjusted in the future.
	
	//add more fields/constants if necessary.
	
	/*
	 * IMPORTANT NOTE:
	 * All methods accessing/modifying obstaclesFound, paths, threads, and currentPos have to be synchronized with same lock. 
	 * Feel free to create any private methods you need.
	 * 
	 * For threads:
	 *   Since this will be multi-thread, I will assign indexes of Thread[] threads array to different tasks to easily access/interrupt
	 *   things on different threads efficiently and to prevent us messing with others' thread
	 *   Let me know if you don't like this or think up with better way to organize
	 *   index 0: Thread that updates obstacles that we see
	 *   index 1: Thread that moves robot in GUI
	 *   index 2: midLineAlgorithm path generator
	 *   index 3: modifiedAStar path generator
	 *   index 4: arcPathAlgorithm path generator
	 *   index 5 and more will be added as we come up with more algorithms.
	 *   
	 *   Pretty sure java GUI is already on its own thread and has a way to access it... correct me if I am wrong.
	 * 
	 * For paths array:
	 *   Again, I will just assign indexes to each path created by different algorithms to keep things organized.
	 *   Let me know if you come up with better idea :)
	 *   index 0: path created by midLine algorithm
	 *   index 1: path created by modifiedAStar algorithm.
	 *   index 2: path created by arcPath algorithm
	 *   index 3 and more will be added as we come up with more algorithms.
	 *   
	 * TODO:
	 *   1) randomObstacleGenerator method - creates 6 obstacles within the arena.
	 *   2) pathGenerator methods - one method per algorithm constructs the field path. Should run on thread and modify path as we see more obstacles.
	 *   3) moveRobot method - should run the thread to display the robot moving with given path and appropriate delays according to robot speed
	 *   4) GUI stuff - displays arena, obstacle, obstacles we see, robot, robot movements. path the robot moved along in real time(On thread)
	 *   5) main method - Runs simulation measure/generate results and either display on GUI or create text file.
	 *   
	 */
	
	public PathPlanSimulator(Position currentPos, Position destination){
		this.currentPos = currentPos;
		this.destination = destination;
		generateObstacles();
		
		/*TODO
		 *setup GUI or anything necessary 
		 */
	}
	
	/*
	 * Generates total 6 obstacles within the arena
	 * Each obstacle is an instance of Position and should be inside the obstacle area.
	 * The coordinate of obstacle will be center of the obstacle. Make sure that obstacle is within the arena with account of diameter.
	 */
	private void generateObstacles(){
		//TODO empty method body
	}
	
	/*
	 * Returns false when given coordinates are not valid
	 */
	private boolean modifyObstacle(Position obstacle, float x_pos, float y_pos){
		if(y_pos > 1.5F && y_pos < 4.44F){ //if inside obstacle area
			float originalX = obstacle.getX();
			float originalY = obstacle.getY();
			if(!obstacle.setX(x_pos) || !obstacle.setY(y_pos)){
				obstacle.setX(originalX);
				obstacle.setY(originalY);
				return false;
			}
			else{
				return true;
			}
		}
		return false;
	}
	
	/*
	 * Runs thread that looks for obstacles and add as we see them.
	 */
	private void findObstacles(){
		//TODO empty method body
	}
	
	/*
	 * Returns false when failed to create a path.
	 * Runs a thread to dynamically change the path as we see more obstacles
	 */
	private boolean midLineAlgorithm(){
		//TODO empty method body
		return false;
	}
	
	/*
	 * Returns false when failed to create a path.
	 * Runs a thread to dynamically change the path as we see more obstacles
	 */
	private boolean modifiedAStar(){
		//TODO empty method body
		return false;
	}
	
	/*
	 * Returns false when failed to create a path.
	 * Runs a thread to dynamically change the path as wee see more obstacles
	 */
	private boolean arcPathAlgorithm(){
		//TODO empty method body
		return false;
	}
	
	/*
	 * Runs thread to display robots moving with given speed, path and obstacles.
	 * We should decide whether we want to show robots on different path created by different algorithms simultaneously or one at a time. 
	 */
	private void moveRobots(){
		//TODO empty method body
	}
	
	//To be done at last
	public static void main(String[] args){
		//TODO empty method body
	}
	
}