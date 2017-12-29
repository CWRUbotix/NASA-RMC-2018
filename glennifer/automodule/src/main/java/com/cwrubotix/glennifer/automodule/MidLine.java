package main.java.com.cwrubotix.glennifer.automodule;

import java.util.HashMap;
import java.util.Arrays;

/**
 * MidLine path planning algorithm.
 * <p>
 * Whenever the robot encounters the obstacle, the robot turns towards the vertical middle guideline that intersects the center of collection bin.\n
 * Detailed description of this algorithm can be found <a href = "https://docs.google.com/a/case.edu/document/d/1sr-W45h1_HqKb4idMiwREjDGKi0yySJnejTE9paQoOs/edit?usp=sharing">here</a>
 * </p>
 * 
 * NOTE: For the current state, this algorithm only works for one way, not for the way back; therefore, 
 * new instance of this class need to be created, and all obstacles that were found should be registered 
 * again as they are re-found to plan for another trip.
 * 
 * @author Seohyun Jung
 *
 */
public class MidLine implements PathFindingAlgorithm{
    
    /** The start position of the path to create*/
    private Position start;
    /** The end position of the path to create*/
    private Position end;
    /** The map of Obstacles found and the locations they were found.*/
    private HashMap<Obstacle, Position> currentPosSet = new HashMap<>();

    @Override
    public Path computePath(Position startPosition, Position endPosition) {
	start = startPosition;
	end = endPosition;
	Path path =  midLine(start, end);
	setAngles(path);
	return path;
    }

    @Override
    public Path computePath(Position currentPos, Obstacle newObstacle) {
	currentPosSet.put(newObstacle, currentPos);
	Path firstHalf = midLine(start, currentPos);
	Path secondHalf = midLine(currentPos, end);
	Path path = firstHalf;
	boolean skipped = false;
	
	for(Position p : secondHalf){
	    if(!skipped){ //skipping first position so that there are no two current position nodes in the path
		skipped = true;
	    }
	    else{
		path.addLast(p);
	    }
	}
	
	setAngles(path);
	return path;
    }
    
    /**
     * Main MidLine algorithm implementation.
     * @param start the start position of the path to create
     * @param end the end position of the path to create
     * @return the Path created by MidLine algorithm
     */
    protected Path midLine(Position start, Position end){
	Obstacle[] obstacles = currentPosSet.keySet().toArray(new Obstacle[0]); //array of found obstacles
	Position current = start;
	Path path = new Path();
	
	while(current != end){
	    path.addLast(current);
	    Arrays.sort(obstacles, Position.getComparatorByDistTo(current)); //sorts the obstacles by the order of encounters
	    Obstacle avoided = null;
	    for(Obstacle obs : obstacles){
		Position temp = getNextPos(current, end, obs);
		if(temp != null){ //If the obstacle needs to be avoided
		    path.addLast(currentPosSet.get(obs));
		    path.addLast(temp);
		    current = temp;
		    avoided = obs;
		    break; //Ends the loop
		}
	    }
	    if(avoided != null)
		remove(avoided, obstacles); //remove the obstacle that is already avoided from the list
	    else{
		current = end; //No more obstacle to avoid
	    }
	}
	path.addLast(end); //Adds destination position
	
	return path;
    }
    
    /**
     * Decides whether the obstacle needs to be avoided to reach end position from the current position and returns
     * the next position to go around the obstacle if it needs to be avoided
     * @param current current position of the robot
     * @param end the destination of the robot
     * @param obs obstacle being evaluated
     * @return next position to go around the obstacle or <code>null</code> if the obstacle can be ignored
     */
    private Position getNextPos(Position current, Position end, Obstacle obs){
	/*Angle between the tangent line of clearance range that intersects current node position and the line between current node and center of Obstacle*/
	double theta = Math.atan((Position.WALL_CLEARANCE() + obs.getRadius()) / current.getDistTo(obs));
	
	/*Absolute angle positions of two tangent lines of clearance ranges that intersects current position*/
	double leftBound = current.getAngleTurnTo(obs) - theta;
	double rightBound = current.getAngleTurnTo(obs) + theta;
	
	if(rightBound > Math.PI * 2) rightBound = rightBound - 2 * Math.PI; // In case the angle bounds
	if(leftBound < 0) leftBound = leftBound + 2 * Math.PI;              // exceed the angle range
	
	double angle = current.getAngleTurnTo(end); // absolute angle position of end node relative to the current node
	
	boolean onTheWay = false; //Marks whether the obstacle is in between the two positions given
	
	if(leftBound < rightBound){ // Normal case
	    if(angle > leftBound && angle < rightBound) onTheWay = false;
	    else onTheWay = true;
	}
	else{ // Special case, when either leftBound or rightBound value exceeded angle range
	    if(angle > rightBound && angle < leftBound) onTheWay = true;
	    else onTheWay = false;
	}
	
	if(!onTheWay) //In this case, there is no need to worry about this obstacle
	    return null;
	
	/*Distance between current Position and to the next Position to go*/
	double dist = Math.sqrt(Math.pow(Position.WALL_CLEARANCE() + obs.getRadius(), 2) + Math.pow(current.getDistTo(obs), 2));
	double x,y;
	
	if(obs.getX() < 0){ //Should turn right
	    x = current.getX() + dist * Math.sin(rightBound);
	    y = current.getY() + dist * Math.cos(rightBound);
	}
	else{ //Should turn left
	    x = current.getX() + dist * Math.sin(leftBound);
	    y = current.getY() + dist * Math.cos(leftBound);
	}
	
	return new Position((float)x, (float)y);
    }
    
    /**
     * Helper method to remove obstacle from the list
     * @param obs obstacle to remove
     * @param obstacles the list manipulated
     */
    private void remove(Obstacle obs, Obstacle[] obstacles){
	for(int i = 0; i < obstacles.length; i++){
	    if(obstacles[i].equals(obs)){
		Obstacle[] newList = new Obstacle[obstacles.length - 1];
		boolean copying = true;
		for(int j = 0; j < newList.length; j++){
		    if(copying){
			newList[j] = obstacles[j];
			if(j + 1 == i) copying = false;
		    }
		    else{
			newList[j] = obstacles[j + 1];
		    }
		}
		return;
	    }
	}
    }
    
    /**
     * Iterates through nodes in the given path and sets up angle fields
     * @param path the path whose nodes needs their angle field set up.
     */
    private void setAngles(Path path){
	if(path == null) return;
	Position previous = path.getPoint(0);
	boolean skipFirst = false;
	for(Position p : path){
	    if(!skipFirst)
		skipFirst = true;
	    else{
		p.setAngle(previous.getAngleTurnTo(p));
	    }
	}
    }
}
