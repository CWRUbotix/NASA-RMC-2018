package main.java.com.cwrubotix.glennifer.automodule;

import java.util.ArrayList;

public class MidLine implements PathFindingAlgorithm{

	private Position start;
	private Position end;
	private Path path = new Path();
	private ArrayList<Obstacle> obstacles;
	private boolean initialized = false;
	private final float obstacleClearance = 0.75F;
	
	public MidLine(Position start, Position end){
		this.start = start;
		this.end = end;
		obstacles = new ArrayList<Obstacle>(6);
	}
	
	public Position getStart(){
		return start;
	}
	
	public void setStart(Position start){
		this.start = start;	}
	
	public Position getEnd(){
		return end;
	}
	
	public void setEnd(Position end){
		this.end = end;
	}
	
	/**
	 * computes the path for robot with given start and end positions.
	 * For midLineAlgorithm, computePath method with startPosition and endPosition input need to be called only once at the beginning
	 * @param startPosition the start position of the path
	 * @param endPosition the end position of the path
	 * @return returns the path created
	 */
	@Override
	public Path computePath(Position startPosition, Position endPosition) {
		if(!initialized) 
			initialized = true; 
		else 
			return path;
		
		setStart(startPosition);
		setEnd(endPosition);
		double angle = startPosition.getAngleTurnTo(endPosition);
		getStart().setAngle(angle);
		getEnd().setAngle(angle);
		path.addFirst(getStart());
		path.addLast(getEnd());
		
		return path;
	}

	@Override
	public Path computePath(Position currentPos, Obstacle newObstacle) {
		if(!initialized)
			computePath(currentPos, getEnd());
		int change = modifyPreviousPath(currentPos);
		for(int i = change; i < path.getPath().size() - 1; i++){
		    path.remove(i);
		}
		path.add(change++, currentPos);
		path.getPath().getLast().setAngle(currentPos.getAngleTurnTo(getEnd()));
		if(!obstacles.contains(newObstacle)){
			obstacles.add(newObstacle);
			Position a,b,c;
			if(currentPos.getX() <= 0){//turn right
				float x = (float)(Math.sqrt((Math.pow(currentPos.getDistTo(newObstacle), 2) - Math.pow(obstacleClearance, 2))));
				double theta = Math.PI / 2 - currentPos.getAngleTurnTo(newObstacle) + currentPos.getAngle();
				a = new Position((float)(currentPos.getX() + x * Math.cos(theta)), 
										  (float)(currentPos.getY() + x * Math.sin(theta)), 
										  0.0F, 0.0F);
				a.setAngle(currentPos.getAngleTurnTo(a));
				x = (float)(Math.sqrt((Math.pow(getEnd().getDistTo(newObstacle), 2) - Math.pow(obstacleClearance, 2))));
				theta = Math.PI / 2 - getEnd().getAngleTurnTo(newObstacle) + currentPos.getAngle();
				c = new Position((float)(currentPos.getX() + x * Math.cos(theta)), 
										  (float)(currentPos.getY() + x * Math.sin(theta)), 
										  0.0F, 0.0F);
				c.setAngle(a.getAngleTurnTo(c));
				double x_pos = a.getX() * Math.tan(Math.PI / 2 - a.getAngle()) - a.getY() - c.getX() * Math.tan(c.getAngle()) + c.getY();
				double y_pos = c.getY() + Math.tan(c.getAngle()) * (x - c.getX());
				b = new Position((float)x_pos, (float)y_pos, a.getAngle(), 0.0F);
			}
			else{//turn left
				float x = (float)(Math.sqrt((Math.pow(currentPos.getDistTo(newObstacle), 2) - Math.pow(obstacleClearance, 2))));
				double theta = 5 * Math.PI / 2 - (currentPos.getAngleTurnTo(newObstacle)) - Math.atan(obstacleClearance / x);
				a = new Position((float)(currentPos.getX() + x * Math.cos(theta)), 
										  (float)(currentPos.getY() + x * Math.sin(theta)), 
										  0.0F, 0.0F);
				a.setAngle(currentPos.getAngleTurnTo(a));
				x = (float)(Math.sqrt((Math.pow(getEnd().getDistTo(newObstacle), 2) - Math.pow(obstacleClearance, 2))));
				theta = 5 * Math.PI / 2 - (getEnd().getAngleTurnTo(newObstacle)) - Math.atan(obstacleClearance / x);
				c = new Position((float)(currentPos.getX() + x * Math.cos(theta)), 
										  (float)(currentPos.getY() + x * Math.sin(theta)), 
										  0.0F, 0.0F);
				c.setAngle(a.getAngleTurnTo(c));
				double x_pos = a.getX() * Math.tan(Math.PI / 2 - a.getAngle()) - a.getY() - c.getX() * Math.tan(c.getAngle()) + c.getY();
				double y_pos = c.getY() + Math.tan(c.getAngle()) * (x - c.getX());
				b = new Position((float)x_pos, (float)y_pos, a.getAngle(), 0.0F);
			}
			//verifying the path
			if(!a.setX(a.getX()) || !a.setY(a.getY()) || !b.setX(b.getX()) || !b.setY(b.getY()) || !c.setX(c.getX()) || !c.setY(c.getY()))
				throw new RuntimeException("failed to create a valid path");
			
			path.add(change++, a);
			path.add(change++, b);
			path.add(change, c);
			path.getPath().getLast().setAngle(c.getAngleTurnTo(getEnd()));
		}
		
		
		return path;
	}
	
	private int modifyPreviousPath(Position currentPos){
		Position previous = null;
		for(Position a : path.getPath()){
		    if(previous != null){
			if(a.getX() < previous.getX()){
			    if(a.getY() < previous.getY()){
				if(currentPos.getX() > a.getX() && currentPos.getX() < previous.getX() && currentPos.getY() > a.getY() && currentPos.getY() < previous.getY()){
				    return path.getPath().indexOf(a);
				}
			    }
			    else{
				if(currentPos.getX() > a.getX() && currentPos.getX() < previous.getX() && currentPos.getY() < a.getY() && currentPos.getY() > previous.getY()){
				    return path.getPath().indexOf(a);
				}
			    }
			}
			else{
			    if(a.getY() < previous.getY()){
				if(currentPos.getX() < a.getX() && currentPos.getX() > previous.getX() && currentPos.getY() > a.getY() && currentPos.getY() < previous.getY()){
				    return path.getPath().indexOf(a);
				}
			    }
			    else{
				if(currentPos.getX() < a.getX() && currentPos.getX() > previous.getX() && currentPos.getY() < a.getY() && currentPos.getY() > previous.getY()){
				    return path.getPath().indexOf(a);
				}
			    }
			}
		    }
		    previous = a;
		}
		return -1; //should never happen;
	    }
	
}