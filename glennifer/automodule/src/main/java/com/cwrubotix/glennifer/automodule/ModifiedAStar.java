package main.java.com.cwrubotix.glennifer.automodule;

import java.util.ArrayList;
import java.util.LinkedList;

/**
 * PathFinder class that uses ModifiedAStar algorithm
 * @author Seohyun Jung
 * 
 * <p>
 * ModifiedAStar algorithm is a traditional AStar search algorithm with account for the fact that
 * not all obstacles in the arena are visible before attaining certain proximity to them. This ModifiedAStar class
 * has methods that can be used to update obstacle locations and modify path accordingly.
 * </p>
 *
 */
public class ModifiedAStar implements PathFindingAlgorithm{
	/** Start position for path planning*/
	private Position startPosition;
	/** End position for path planning*/
	private Position endPosition;
	/** Stores all the nodes created*/
	private LinkedList<AStarNode> nodes;
	private ArrayList<Obstacle> obstacles;
	
	public ModifiedAStar(Position currentPos, Position endPosition){
		this.startPosition = currentPos;
		this.endPosition = endPosition;
	}
	
	public Position getStartPosition(){
		return startPosition;
	}
	
	protected void setStartPosition(Position startPosition){
		this.startPosition = startPosition;
	}
	
	public Position getEndPosition(){
		return endPosition;
	}
	
	protected void setEndPosition(Position endPosition){
		this.endPosition = endPosition;
		for(AStarNode node : nodes){
			node.setVisited(false);
			node.computeHeruistic(endPosition);
		}
	}
	
	@Override
	public Path computePath(Position currentPos, Position destination) {
		// TODO Auto-generated method stub
		return null;
	}
	
	@Override
	public Path computePath(Position currentPos, Obstacle newObstacle) {
		// TODO Auto-generated method stub
		return null;
	}
	
	private class AStarNode extends Position implements Comparable<AStarNode>{
		private LinkedList<AStarNode> connected;
		private float heruistic;
		private AStarNode previous;
		private boolean visited = false;
		
		private AStarNode(float x_pos, float y_pos){
			super(x_pos, y_pos, 0.0F, 0.0F);
			computeHeruistic(getEndPosition());
		}
		
		private AStarNode(float x_pos, float y_pos, LinkedList<AStarNode> connected){
			this(x_pos, y_pos);
			this.connected = connected;
		}
		
		private AStarNode(Position pos){
			this(pos.getX(), pos.getY());
		}
		
		private boolean isVisited(){
			return visited;
		}
		
		private void setVisited(boolean visited){
			this.visited = visited;
		}
		
		private float getHeruistic(){
			return heruistic;
		}
		
		private void computeHeruistic(Position destination){
			this.heruistic = getDistTo(destination);
		}
		
		public int compareTo(AStarNode a){
			if(getHeruistic() - a.getHeruistic() < 0)
				return -1;
			else if(getHeruistic() - a.getHeruistic() > 0)
				return 1;
			else
				return 0;
		}
	}
}