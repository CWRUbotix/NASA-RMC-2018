package main.java.com.cwrubotix.glennifer.automodule;

import java.util.Iterator;
import java.util.LinkedList;

public class Path implements Iterable<Position>{
	private LinkedList<Position> path;
	private final Position start;
	private final Position destination;
	private int currentSubDestination;
	private int numPoints;
	
	public Path(Position start, Position destination){
		this.start = start;
		this.destination = destination;
		path = new LinkedList<Position>();
		add(start);
		add(destination);
		currentSubDestination = 1;
	}
	
	private void add(Position point){
		getPath().add(point);
		numPoints++;
	}
	
	protected LinkedList<Position> getPath(){
		return path;
	}
	
	/*
	 * NOTE: this method returns path in array of Positions form
	 * This method is for testing/debugging purpose only.
	 * Use this to check whether your path generator method is working properly.
	 * Should not be used for other purpose and will be removed once PathPlanSimulator is done.
	 */
	public Position[] getList(){
		return (Position[]) getPath().toArray();
	}
	
	protected Position getStart(){
		return start;
	}
	
	protected Position getDestination(){
		return destination;
	}
	
	public Position getPoint(int index){
		return path.get(index);
	}
	
	public boolean add(int index, Position point){
		if(index != 0 && index != numPoints - 1){ //Should not modify first and last points.
			getPath().add(index, point);
			numPoints++;
			return true;
		}
		return false;
	}
	
	public boolean remove(int index){
		if(index == 0 || index == numPoints - 1){ //Should never remove start and destination
			return false;
		}
		getPath().remove(index);
		numPoints--;
		return true;
	}
	
	public boolean remove(Position point){
		if(!point.equals(getStart()) && !point.equals(getDestination())){ //Should never remove start and destination.
			if(getPath().remove(point)){
				numPoints--;
				return true;
			}
		}
		return false;
	}
	
	/*
	 * Should be called by PathPlanSimulator before running the test to ensure all paths to be tested
	 * have same start point and same destination.
	 */
	public static boolean verifySamePathGoals(Path[] paths){
		Position start = paths[0].getStart();
		Position destination = paths[0].getDestination();
		boolean valid = true;
		for(Path path : paths){
			if(!path.getStart().equals(start) || !path.getDestination().equals(destination))
				valid = false;
		}
		return valid;
	}
	
	/*
	 * May or may not be used..?
	 * I thought it could be useful if we modify iterator so that we can still use for each loop even when the path is dynamically changing.
	 * Will only work if for each loop iterating this has enough delay when moving between points and properly synchronized.
	 */
	public Iterator<Position> iterator(){
		
		return new Iterator<Position>(){
			
			@Override
			public boolean hasNext() {
				if(currentSubDestination < numPoints)
					return true;
				return false;
			}

			@Override
			public Position next() {
				return getPath().get(currentSubDestination++);
			}
			
		};
	}
}
