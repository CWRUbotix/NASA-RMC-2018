package main.java.com.cwrubotix.glennifer.automodule;

import java.util.HashMap;
import java.util.Arrays;

public class MidLine implements PathFindingAlgorithm{
    
    private Position start;
    private Position end;
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
    
    protected Path midLine(Position start, Position end){
	Obstacle[] obstacles = currentPosSet.keySet().toArray(new Obstacle[0]);
	Position current = start;
	Path path = new Path();
	
	while(current != end){
	    path.addLast(current);
	    Arrays.sort(obstacles, Position.getComparatorByDistTo(current));
	    Obstacle avoided = null;
	    for(Obstacle obs : obstacles){
		Position temp = getNextPos(current, end, obs);
		if(temp != null){
		    path.addLast(currentPosSet.get(obs));
		    path.addLast(temp);
		    current = temp;
		    avoided = obs;
		    break;
		}
	    }
	    if(avoided != null)
		remove(avoided, obstacles);
	    else{
		current = end;
	    }
	}
	path.addLast(end);
	
	return path;
    }
    
    private Position getNextPos(Position start, Position end, Obstacle obs){
	/*Angle between the tangent line of clearance range that intersects start node position and the line between start node and center of Obstacle*/
	double theta = Math.atan((Position.WALL_CLEARANCE() + obs.getDiameter() / 2) / start.getDistTo(obs));
	
	/*Absolute angle positions of two tangent lines of clearance ranges that intersects start position*/
	double leftBound = start.getAngleTurnTo(obs) - theta;
	double rightBound = start.getAngleTurnTo(obs) + theta;
	
	if(rightBound > Math.PI * 2) rightBound = rightBound - 2 * Math.PI; // In case the angle bounds
	if(leftBound < 0) leftBound = leftBound + 2 * Math.PI;              // exceed the angle range
	
	double angle = start.getAngleTurnTo(end); // absolute angle position of end node relative to the start node
	
	boolean onTheWay = false;
	
	if(leftBound < rightBound){ // Normal case
	    if(angle > leftBound && angle < rightBound) onTheWay = false;
	    else onTheWay = true;
	}
	else{ // Special case, when either leftBound or rightBound value exceeded angle range
	    if(angle > rightBound && angle < leftBound) onTheWay = true;
	    else onTheWay = false;
	}
	
	if(!onTheWay)
	    return null;
	
	double dist = Math.sqrt(Math.pow(Position.WALL_CLEARANCE() + obs.getDiameter() / 2, 2) + Math.pow(start.getDistTo(obs), 2));
	double x,y;
	
	if(obs.getX() < 0){
	    x = start.getX() + dist * Math.sin(rightBound);
	    y = start.getY() + dist * Math.cos(rightBound);
	}
	else{
	    x = start.getX() + dist * Math.sin(leftBound);
	    y = start.getY() + dist * Math.cos(leftBound);
	}
	
	return new Position((float)x, (float)y);
    }
    
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
