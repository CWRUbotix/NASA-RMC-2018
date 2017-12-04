package main.java.com.cwrubotix.glennifer.automodule;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;

/**
 * PathFinder class that uses ModifiedAStar algorithm
 * @author Seohyun Jung
 * 
 * <p>
 * ModifiedAStar algorithm is a traditional AStar search algorithm with account for the fact that
 * not all obstacles in the arena are visible before attaining certain proximity to them. 
 * This ModifiedAStar class has methods that can be used to update obstacle locations and modify path accordingly.
 * </p>
 *
 */
public class ModifiedAStar implements PathFindingAlgorithm{
    /** Start position for path planning*/
    private AStarNode startPosition;
    /** End position for path planning*/
    private AStarNode endPosition;
    /** Stores all the nodes created*/
    private LinkedList<AStarNode> nodes;
    /** Last path created*/
    private Path path;
    /** Stores obstacles within the arena*/
    private ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>(6);
    
    private final float obstacleClearance = 0.75F;
    
    /**
     * Creates new ModifiedAStar PathFindingAlgorithm with current position and end position
     * @param currentPos current position of the robot
     * @param endPosition destination of the robot
     */
    public ModifiedAStar(Position currentPos, Position endPosition){
	AStarNode start = new AStarNode(currentPos);
	AStarNode end = new AStarNode(endPosition);
	nodes.add(start);
	nodes.add(end);
	start.connect(end);
	end.connect(start);
	startPosition = start;
	this.endPosition = end;
    }
    
    /**
     * returns start position of the path
     * @return start position of the path
     */
    public Position getStartPosition(){
	return startPosition;
    }
    
    /**
     * sets start position of the path if needed to be changed
     * @param startPosition the start position of the robot
     */
    protected void setStartPosition(AStarNode startPosition){
	this.startPosition = startPosition;
	if(!nodes.contains(startPosition)){
	    nodes.add(startPosition);
	    connectToAll(startPosition);
	}
	for(AStarNode node : nodes){
	    node.setVisited(false);
	    node.setPrevious(null);
	    node.computeHeruistic(endPosition);
	    path = new Path();
	}
    }
    
    /**
     * gets destination position of the path
     * @return destination position of the path
     */
    public Position getEndPosition(){
	return endPosition;
    }
    
    /**
     * sets destination position of the path
     * @param endPosition destination position of the path
     */
    protected void setEndPosition(AStarNode endPosition){
	this.endPosition = endPosition;
	if(!nodes.contains(endPosition)){
	    nodes.add(endPosition);
	    connectToAll(endPosition);
	}
    }
    
    
    @Override
    public Path computePath(Position startPos, Position destination) {
	AStarNode current = new AStarNode(startPos);
	if(!nodes.contains(current)){
	    nodes.add(current);
	    connectToAll(current);
	}
	setStartPosition(current);
	setEndPosition(new AStarNode(destination));
	
	AStarNode currentNode = (AStarNode)getStartPosition();
	while(!currentNode.equals(getEndPosition())){
	    currentNode.setVisited(true);
	    AStarNode[] connects = new AStarNode[currentNode.getConnected().size()];
	    Arrays.sort(currentNode.getConnected().toArray(connects));
	    int next = -1;
	    for(int i = 0; i < connects.length && next == -1; i++){
		if(currentNode.updateDistances())
		    next = i;
	    }
	    if(next == -1){
		currentNode = currentNode.getPrevious();
		if(currentNode == null)
		    throw new RuntimeException("Failed to create a path");
	    }
	    else{
		connects[next].setPrevious(currentNode);
		currentNode = connects[next];
	    }
	}
	
	Path path = new Path();
	while(!currentNode.equals(getStartPosition())){
	    path.addFirst(currentNode);
	    currentNode = currentNode.getPrevious();
	}
	path.addFirst(currentNode);
	this.path = path;
	return path;
    }
    
    
    @Override
    public Path computePath(Position currentPos, Obstacle newObstacle) {
	AStarNode current = new AStarNode(currentPos);
	if(!obstacles.contains(newObstacle)){
	    obstacles.add(newObstacle);
	    addNodesAroundObs(newObstacle);
	    disconnectFromEnd();
	    connectToAll((AStarNode)getEndPosition());
	    if(!nodes.contains(current)){
		nodes.add(current);
		connectToAll(current);
	    }
	    modifyPreviousPath(current);
	    AStarNode currentNode = current;
	    while(!currentNode.equals(getEndPosition())){
		currentNode.setVisited(true);
		AStarNode[] connects = new AStarNode[currentNode.getConnected().size()];
		Arrays.sort(currentNode.getConnected().toArray(connects));
		int next = -1;
		for(int i = 0; i < connects.length && next == -1; i++){
		    if(currentNode.updateDistances())
			next = i;
		}
		if(next == -1){
		    currentNode = currentNode.getPrevious();
		    if(currentNode == null)
			throw new RuntimeException("Failed to create a path");
		}
		else{
		    connects[next].setPrevious(currentNode);
		    currentNode = connects[next];
		}
	    }
	    
	    Path path = new Path();
	    while(!currentNode.equals(getStartPosition())){
		path.addFirst(currentNode);
		currentNode = currentNode.getPrevious();
	    }
	    path.addFirst(currentNode);
	    this.path = path;
	    return path;
	}
	else
	    return this.path;
    }
    
    private void modifyPreviousPath(AStarNode currentPos){
	AStarNode previous = null;
	for(Position a : path.getPath()){
	    if(previous != null){
		if(a.getX() < previous.getX()){
		    if(a.getY() < previous.getY()){
			if(currentPos.getX() > a.getX() && currentPos.getX() < previous.getX() && currentPos.getY() > a.getY() && currentPos.getY() < previous.getY()){
			    currentPos.setPrevious(previous);
			    ((AStarNode)a).setPrevious(currentPos);
			    return;
			}
		    }
		    else{
			if(currentPos.getX() > a.getX() && currentPos.getX() < previous.getX() && currentPos.getY() < a.getY() && currentPos.getY() > previous.getY()){
			    currentPos.setPrevious(previous);
			    ((AStarNode)a).setPrevious(currentPos);
			    return;
			}
		    }
		}
		else{
		    if(a.getY() < previous.getY()){
			if(currentPos.getX() < a.getX() && currentPos.getX() > previous.getX() && currentPos.getY() > a.getY() && currentPos.getY() < previous.getY()){
			    currentPos.setPrevious(previous);
			    ((AStarNode)a).setPrevious(currentPos);
			    return;
			}
		    }
		    else{
			if(currentPos.getX() < a.getX() && currentPos.getX() > previous.getX() && currentPos.getY() < a.getY() && currentPos.getY() > previous.getY()){
			    currentPos.setPrevious(previous);
			    ((AStarNode)a).setPrevious(currentPos);
			    return;
			}
		    }
		}
	    }
	    previous = (AStarNode) a;
	}
    }
    
    private boolean isValidConnection(AStarNode start, AStarNode end){
	if(start.isAroundObstacle() && !end.isAroundObstacle())
	    return isValidConnection(end, start);
	boolean result = true;
	for(Obstacle obs : obstacles){
	    double theta = Math.asin(obstacleClearance / start.getDistTo(obs));
	    double alpha = start.getAngleTurnTo(obs);
	    if(start.getAngleTurnTo(end) >= alpha - theta && start.getAngleTurnTo(end) <= alpha + theta){
		result = false;
	    }
	}
	return result;
    }
    
    private void addNodesAroundObs(Obstacle newObstacle){
	for(double i = 0; i < 2 * Math.PI; i = i + Math.PI / 4){
	    AStarNode temp = new AStarNode((float)(newObstacle.getX() + obstacleClearance * Math.cos(i)), (float)(newObstacle.getY() + obstacleClearance  * Math.sin(i)));
	    nodes.add(temp);
	    temp.setAroundObstacle();
	    connectToAll(temp);
	}
    }
    
    private void connectToAll(AStarNode node){
	for(AStarNode other : nodes){
	    if(isValidConnection(node, other)){
		node.connect(other);
		other.connect(node);
	    }
	}
    }
    
    private void disconnectFromEnd(){
	for(AStarNode node : ((AStarNode)getEndPosition()).getConnected()){
	    node.getConnected().remove((AStarNode)getEndPosition());
	}
	((AStarNode)getEndPosition()).connected = null;
    }
    
    /**
     * AStarNode represents each data point of AStar algorithm
     */
    private class AStarNode extends Position implements Comparable<AStarNode>{
	/** list of AStarNodes that this node is connected to*/
	private LinkedList<AStarNode> connected;
	/** heruistic of the Node. Heruistic is defined to be distance between this node to destination node*/
	private float heruistic;
	/** stores the last node that path came from*/
	private AStarNode previous;
	/** indicates whether this node was visited or not*/
	private boolean visited = false;
	/** indicates whether hits node is around the obstacle*/
	private boolean aroundObstacle = false;
	/** stores distance of shortest path found so far to this node*/
	private float distance = 0;
	
	/** 
	 * Creates new AStarNode with coordinates given
	 * @param x_pos x coordinate of the Node
	 * @param y_pos y coordinate of the Node
	 */
	private AStarNode(float x_pos, float y_pos){
	    super(x_pos, y_pos, 0.0F, 0.0F);
	    computeHeruistic(getEndPosition());
	}
	
	/**
	 * Creates new AStarNode with coordinates given and nodes that are connected to the node creating
	 * @param x_pos x coordinate of the Node
	 * @param y_pos y coordinate of the Node
	 * @param connected list of nodes that this node is connected to
	 */
	private AStarNode(float x_pos, float y_pos, LinkedList<AStarNode> connected){
	    this(x_pos, y_pos);
	    this.connected = connected;
	}
	
	/**
	 * Creates new AStarNode with Position instance indicating position of the node
	 * @param pos position instance that represents coordinate of the node
	 */
	private AStarNode(Position pos){
	    this(pos.getX(), pos.getY());
	}
	
	/**
	 * Checks whether the node has been visited by pathfinding algorithm
	 * @return whether the node has been visited by pathfinding algorithm
	 */
	private boolean isVisited(){
	    return visited;
	}
	
	/**
	 * sets the node as visited or not visited
	 * @param visited whether the node has been visited
	 */
	private void setVisited(boolean visited){
	    this.visited = visited;
	}
	
	/**
	 * returns the heruistic of the node
	 * @return the heruistic of the node
	 */
	private float getHeruistic(){
	    return heruistic;
	}	
	
	private float getDistance(){
	    return distance;
	}

	/**
	 * takes in destination position and computes heruistic of the node
	 * @param destination
	 */
	private void computeHeruistic(Position destination){
	    this.heruistic = getDistTo(destination);
	}
	
	private boolean isAroundObstacle(){
	    return aroundObstacle;
	}
	
	private void setAroundObstacle(){
	    aroundObstacle = true;
	}
	
	private AStarNode getPrevious(){
	    return previous;
	}
	
	private void setPrevious(AStarNode previous){
	    this.previous = previous;
	}
	
	private LinkedList<AStarNode> getConnected(){
	    return connected;
	}
	
	private void connect(AStarNode node){
	    connected.add(node);
	}
	
	private boolean updateDistances(){
	    AStarNode[] connects = (AStarNode[]) connected.toArray();
	    Arrays.sort(connects);
	    for(AStarNode node : connects){
		if(!node.isVisited()){
		    node.distance = getDistTo(node);
		    return true;
		}
		else if(node.getDistance() > getDistance() + getDistTo(node)){
		    node.distance = getDistance() + getDistTo(node);
		    return true;
		}
	    }
	    return false;
	}
	
	/**
	 * compares two AStarNodes with its heruistic
	 * @return negative when current instance has higher (lower value) heruistic
	 */
	@Override
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