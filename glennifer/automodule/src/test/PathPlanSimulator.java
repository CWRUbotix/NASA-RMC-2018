package test;

import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import java.lang.Thread;
import java.util.Arrays;
import java.util.Comparator;
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
	private final Position destination;
	private Position currentPos;
	private Thread obstacleDectection;
	private Thread robotMoving;
	private Path[] paths = new Path[5];
	
	/*Thread Locks*/
	private final Object LockCurrentPos = new Object();
	private final Object LockPath = new Object();
	
	/*Constants:*/
	private final float STRAIGHT_SPEED = 0.0F;  //Will be filled when
	private final float TURNING_SPEED = 0.0F;   //Locomotion gives us numbers.
	private final float ROBOT_WIDTH = 0.75F;  // Will be modified whenever
	private final float ROBOT_LENGTH = 1.0F;  // Estimate on robot size changes
	private final float OBSTACLE_SIZE = 0.3F; //In diameter, Assume all obstacles are max-sized
	private final float KINECT_RANGE = 2.0F; //How far we can see. Will be tested and adjusted in the future.
	private final float OBSTACLE_AREA_HEIGHT = 2.94F;
	private final float SAFE_AREA_HEIGHT = 1.5F;
	
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
	 *   
	 *   Fields for threads:
	 *   obstacleDetection: Thread that updates obstacles that we see
	 *   robotMoving: Thread that moves robot in GUI
	 *   
	 *   Pretty sure java GUI is already on its own thread and has a way to access it... correct me if I am wrong.
	 * 
	 * For paths array:
	 *   Again, I will just assign indexes to each path created by different algorithms to keep things organized.
	 *   Let me know if you come up with better idea :)
	 *   index 0: path created by midLine algorithm
	 *   index 1: path created by modifiedAStar algorithm.
	 *   index 2: path created by arcPath algorithm
	 *   index 3: path created by Diijkstra algorithm
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
	
	public PathPlanSimulator(Position currentPos, Position destination){
		this.currentPos = currentPos;
		this.destination = destination;
		generateObstacles();
		
		/*TODO
		 *setup GUI or anything necessary 
		 */
	}
	
	private static Comparator<Position> getComparatorByDistToCurrentPos(final PathPlanSimulator simulator){
		return new Comparator<Position>(){
			public int compare(Position a, Position b){
				float diffInDist = a.getDistTo(simulator.currentPos) - b.getDistTo(simulator.currentPos);
				if(diffInDist < 0) return -1;
				else if(diffInDist == 0) return 0;
				else return 1;
			}
		};
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
		Arrays.sort(obstacles, PathPlanSimulator.getComparatorByDistToCurrentPos(this));
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
				Arrays.sort(obstacles, PathPlanSimulator.getComparatorByDistToCurrentPos(this));
				return true;
			}
		}
		return false;
	}
	
	/*
	 * Runs thread that looks for obstacles and add as we see them.
	 */
	private void findObstacles(){
		final PathPlanSimulator simulator = this;
		this.obstacleDectection = new Thread(new Runnable(){
			public void run(){
				synchronized(LockCurrentPos){
					while(!currentPos.equals(destination)){
						try {
							LockCurrentPos.wait();
						} catch (InterruptedException e) {
							e.printStackTrace();
						}

						findObstacles();

						synchronized(LockPath){
							if(!simulator.midLineAlgorithm())
								throw new FailedToCreatePathException("Mid Line Algorithm Failed");
							else if(!simulator.modifiedAStar())
								throw new FailedToCreatePathException("Modified AStar Algorithm Failed.");
							else if(!simulator.arcPathAlgorithm())
								throw new FailedToCreatePathException("Arc Path Algorithm Failed");
							//ADD Diijkstra
							else
							  LockPath.notify();
						}
						LockCurrentPos.notify();
					}
				}
			}
			
			private boolean foundObstacles(){
				Position[] obstaclesFound = new Position[6];
				for(int i = 0; i < obstacles.length; i++){
					if(obstacles[i].getDistTo(currentPos) < KINECT_RANGE)
						obstaclesFound[i] = obstacles[i];
					}
				for(int i = 0; i < obstaclesFound.length; i ++){
					if(!obstaclesFound[i].equals(simulator.obstaclesFound[i])){ // Found new obstacle.
						Arrays.sort(obstaclesFound, PathPlanSimulator.getComparatorByDistToCurrentPos(simulator));
						simulator.obstaclesFound = obstaclesFound;
						return true;
					}
				}
				return false;
			}
		});
		this.obstacleDectection.run();
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
	 * STILL UNDER PROGRESS
	 */
	private void moveRobots(){
		this.robotMoving = new Thread(new Runnable(){
			public void run(){
				LockCurrentPos.notify();
				while(!currentPos.equals(destination)){
					if(paths[0] == null){
						try {
							LockPath.wait();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					//TODO Every once in a while, call wait method and notify obstacle finding thread.
				}
			}
		});
		robotMoving.run();
	}
	
	//To be done at last
	public static void main(String[] args){
		//TODO empty method body
	}
	
	private class FailedToCreatePathException extends RuntimeException{

		private static final long serialVersionUID = 1L;

		public FailedToCreatePathException(String message){
			super(message);
		}
	}
	
}