//import static org.junit.jupiter.api.Assertions.*;
//
//import org.junit.jupiter.api.Test;
//
//import javafx.scene.shape.Path;
package main.java.com.cwrubotix.glennifer.automodule;

import java.lang.Object;

/**
 * Data Type implementing the ArcPath algorithm for generating a path through an arena.
 * Algorithm is designed for obstacles of known size and shape but unknown positon. 
 * Designed to be updated as obstacles are spotted.
 * Designed for a robot larger than the obstacles.
 * @author Andrea
 *
 */


public class ArcPath implements PathFindingAlgorithm

{
	private Position target;	 //target position, to be determined outside algorithm
	private Position robot;  //robot starting position, updated when new obstacles added;
	private Position start; // where robot starts
	private Position[] obstacles; //holds obstacle position, up to 6
	private int numObstacles;
	private Path path;
	private float clearance = 0.3F;
	
	private static final float ROBOT_WIDTH = 0.75F; //tenatively set to .75m will obv change when we know the robot's clearance or decide simulation clearance
	
	/*
	 * Constructor, I can add more but I dont think it's really necessary
	 */
	public ArcPath(Position r, Position t) //primary constructor, takes 2 positions, robot position and target position - no known obstacles
	{
		target = t;
		robot = r;
		obstacles = new Position[6];
		numObstacles = 0;
		path = new Path();
		path.add(0,robot);
		path.add(1,target);
	}
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////// Getters and Setters
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	public Position getTarget() 
	{
		return target;
	}
	public Position getStart()
	{
		return robot;
	}
//	public Position getPath()
//	{
//		generatePath();
//		return path;
//	}	
	public Position[] getObstacles()
	{
		return obstacles;
	}
	public void setTarget(Position t)
	{
		target = t;
	}
	public void setStart(Position s)
	{
		robot = s;
	}
	public void setPath(Path p)
	{
		path = p;
	}
	public void setObstacles(Position[] obst)
	{
		obstacles = obst;
	}
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////// PathFindingAlgorithm Methods
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	public Path computePath(Position startPosition, Position endPosition)
	{
		target = endPosition;
		robot = startPosition;
		start = startPosition;
		calcPath(startPosition,endPosition);	
		
		return path;
	}
	
	public Path computePath(Position currentPos, Obstacle newObstacle)
	{
		//update obstacle list
		obstacles[numObstacles] = newObstacle;
		numObstacles ++;
		
		robot = currentPos;
		
		calcPath(currentPos,target);
		
		return path;
	}
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////// Internal Methods
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
	
	private void calcPath(Position start, Position end)
	{	
		path.add(0,start);
		path.add(1,end);
		Position[] checkpoints = getCheckpoints();

		while(!(numObstacles == 0|| obstOnPath())) //while there are any obstacles on the path
		{
			boolean[] onpath = new boolean[6];
//			fill(onpath,false);
			
			//check if each obstacle is on the path
			for(int s=1;s<=numObstacles;s++)
			{
				if(onPath(obstacles[s]))
					onpath[s-1] = true;
			}
			//now I have an boolean array with the true values indicating the corresponding obstacle is on the path
			//check if any obstacles on path overlap with other obstacles, obstacles that overlap count as 
			for(int s=1;s<numObstacles;s++)
			{
				
			}
			//find point on the large obst/obstacles overlapping closest to center
			//generate tangent arc to that point
			//adjust end position to be whatever point on the horiz line containing the original end point is also contained in the vertical line containing the overlap point
			//check to see if there are any obstacles on the new line and do this all over 
			
		}

		
	
	}
	
	private Position[] getCheckpoints()
	{
		//returns positon array containing the points closest to the centerline of the arena - the y-axis
		Position[] positions = [6];
		//need to generate an x and y coordinate from the center coord of an obstacle.
		//r = .3 - 3cm
		for(s=1;s<= numObstacles; s++)
		{
			Position obst = obstacles[s];
			if(obst.getX()>0)
			{
				positions[s] = new Position(obst.getX()-clearance,obst.getY(),0,0)
			}
			else if(obst.getX()<0)
			{
				positions[s] = new Position(obst.getX()+clearance,obst.getY(),0,0)
			}
			else if(robot.getX()>0)
			{
				positions[s] = new Position(obst.getX()-clearance,obst.getY(),0,0)
			}
			else(robot.getY()<0)
			{
				positions[s] = new Position(obst.getX()-clearance,obst.getY(),0,0)
			}	
		}
		
	}
	
	
	private boolean onPath(Position p) //checks if object is on path or within clearance (robot width) of path
	{
		
	}
	private boolean obstOnPath()
	{
		boolean flag = false;
		for(s=1;s<=numObstacles;s++)
		{
			if(onPath(obstacles[s]))
				flag = true;
		}
		return flag;
	}
	
	private boolean pointCollision(Position r, Position o)
	{
		float xr = r.getX(); 
		float yr = r.getY();
		float xo = o.getX();
		float yo = o.getY();
		
		float clearance = ROBOT_WIDTH/2;
		float objectClearance = (float) Math.sqrt( Math.pow((xo-xr), 2) + Math.pow((yo-yr), 2));
		
		return objectClearance <= clearance;	
	}
	private void generateCircle()
	{
		float x1 = obstacle x pos
		float y1 = obstacle y pos
		float x2 = robot x pos
		float y2 = robot y pos
		float x3 = tangent point x
		float y3 = tangent point y
		
		/*
		 * need to calculate the x and y of the center of the tangent circle that the robot will travel on. Formula is (x1-x)^2+(y1-y)^2=(r1+r)^2   unknown: x, y, r
		 * r = +sqrt((x-x2)^2+(y-y2)^2)  unknown: x, y, r
		 * (x1-x)^2+(y1+y2)^2  = (r1+ sqrt((x-x2)^2+(y-y2)^2)^2  unknown: x2, y2    I still need one more equation - some sort of relationship aaaaaa
		 * okay so we know two points on the surface of this circle x2,y2 and x3,y3
		 * x3,y3 are the points on the surface of the circle on the tangent line that is vertical - parallel to the arena sides, need to be calculated
		 * r2= +sqrt((x-x3)^2 + (y-y3)^2)
		 * 
		 * so: equation that will allow us to calculate the centerpoint of the circle we will travel on
		 * formula of circle we are looking for is (x2-x)^2+(y2-y)^2 = r^2 aw shit cant have one half of the circle eqn be the exact same as the other half uhhhhhshshsh
		 * */
		
		
	}

	
}