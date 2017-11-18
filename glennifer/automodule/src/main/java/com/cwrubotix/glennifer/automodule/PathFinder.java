package main.java.com.cwrubotix.glennifer.automodule;


import java.util.ArrayList;


/**
 * Unfinished
 * @param <T> Path finding algorithm
 */
public class PathFinder<T extends PathFindingAlgorithm> {
    private T pathFinder;
    private ArrayList<Obstacle> obstacles;
    private Position start;
    private Position destination;
    private Position currentPos;
    private Path path;

    public PathFinder(T pathFinder, Position start, Position destination) {
        this.obstacles = new ArrayList<Obstacle>(6);
    	this.pathFinder = pathFinder;
        this.start = start;
        this.destination = destination;
        this.path = pathFinder.computePath(start, destination);
    }

    public Path getPath() {
    	return path;
    }
    
    public Position getStart(){
    	return start;
    }
    
    public void setStart(Position start){
    	this.start = start;
    }
    
    public Position getDestination(){
    	return destination;
    }
    
    public void setDestination(Position destination){
    	this.destination = destination;
    }
    
    public Position getCurrentPos(){
    	return currentPos;
    }
    
    protected void setCurrentPos(Position currentPos){
    	this.currentPos = currentPos;
    }

    public void registerObstacle(Obstacle obstacle) {
    	if(!obstacles.contains(obstacle)){
    		obstacles.add(obstacle);
    		this.path = pathFinder.computePath(currentPos, obstacle);
    	}
    }
}
