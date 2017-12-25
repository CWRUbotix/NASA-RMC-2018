package main.java.com.cwrubotix.glennifer.automodule;

import java.util.LinkedList;
import java.util.ArrayList;

public class ModifiedAStar implements PathFindingAlgorithm{
    
    private ArrayList<Obstacle> obstacles = new ArrayList<>(6);
    private LinkedList<AStarNode> nodes = new LinkedList<>();
    private AStarNode start;
    private AStarNode end;
    
    public ArrayList<Obstacle> getObstacles(){
	return obstacles;
    }
    
    public LinkedList<AStarNode> getNodes(){
	return nodes;
    }
    
    @Override
    public Path computePath(Position startPosition, Position endPosition) {
	start = new AStarNode(startPosition);
	start.setAngle(startPosition.getAngle());
	end = new AStarNode(endPosition);
	start.connect(end);
	end.connect(start);
	getNodes().add(start);
	getNodes().add(end);
	Path result = aStar(start, end);
	setAngles(result);
	return result;
    }

    @Override
    public Path computePath(Position currentPos, Obstacle newObstacle) {
	AStarNode cp = new AStarNode(currentPos);
	getObstacles().add(newObstacle);
	createNodes(newObstacle);
	getNodes().add(cp);
	connectToAll();
	
	Path firstHalf = aStar(start, cp);
	Path secondHalf = aStar(cp, end);
	
	Path result = firstHalf;
	
	boolean skipped = false;
	for(Position p : secondHalf){
	    if(!skipped){
		skipped = true;
	    }
	    else{
		result.addLast(p);
	    }
	}
	setAngles(result);
	return result;
    }
    
    private Path aStar(AStarNode start, AStarNode end){
	resetVisited();
	resetFound();
	resetDistances();
	updateHeruistics(end);
	
	AStarNode current = start;
	current.updateDist(0.0F);
	ArrayList<AStarNode> openSet = new ArrayList<>(getNodes().size());
	ArrayList<AStarNode> closedSet = new ArrayList<>(getNodes().size());
	openSet.add(start);
	start.setFound(true);
	while(!openSet.isEmpty()){
	    if(current.equals(end))
		return createPath(end);
	    openSet.remove(current);
	    closedSet.add(current);
	    current.setVisited(true);
	    updateDistances(current);
	    for(AStarNode neighbor : current.getConnections()){
		if(!neighbor.isVisited() && !neighbor.found()){
		    openSet.add(neighbor);
		    neighbor.setFound(true);
		}
	    }
	    current = getMinFScore(openSet);
	}
	
	throw new RuntimeException("Failed to create a path");
    }
    
    private AStarNode getMinFScore(ArrayList<AStarNode> openSet){
	AStarNode save = null;
	for(AStarNode neighbor : openSet){
	    if(save == null){
		save = neighbor.getMinFScore();
	    }
	    else{
		AStarNode temp = neighbor.getMinFScore();
		if(save.getFScore() > temp.getFScore())
		    save = temp;
	    }
	}
	return save;
    }
    
    private void updateDistances(AStarNode current){
	for(AStarNode node : current.getConnections()){
	    float tempGScore = current.getDist() + current.getDistTo(node);
	    if(node.getDist() > tempGScore){
		node.updateDist(tempGScore);
		node.setPrevious(current);
	    }
	}
    }
    
    private void createNodes(Obstacle obs){
	for(int i = 0; i < 6; i++){
	    double angle = Math.PI * i / 3;
	    float clearance = Position.WALL_CLEARANCE() + obs.getDiameter();
	    getNodes().add(new AStarNode((float)(obs.getX() + clearance * Math.cos(angle)), (float)(obs.getY() + clearance * Math.sin(angle))));
	}
    }
    
    private void connectToAll(){
	for(AStarNode node : getNodes()){
	    node.resetConnection();
	    for(AStarNode connect : getNodes()){
		if(isValid(node, connect)){
		    node.connect(connect);
		}
	    }
	}
    }
    
    private boolean isValid(AStarNode start, AStarNode end){
	boolean check = true;
	for(Obstacle obs : getObstacles()){
	    if(isOnTheWay(start, end, obs))
		check = false;
	}
	return check;
    }
    
    private void updateHeruistics(AStarNode end){
	for(AStarNode node : nodes){
	    node.setHeruistic(end);
	}
    }
    
    private void resetVisited(){
	for(AStarNode node : getNodes()){
	    node.setVisited(false);
	}
    }
    
    private void resetFound(){
	for(AStarNode node : getNodes()){
	    node.setFound(false);
	}
    }
    
    private void resetDistances(){
	for(AStarNode node : getNodes()){
	    node.updateDist(Float.POSITIVE_INFINITY);
	}
    }
    
    private boolean isOnTheWay(AStarNode start, AStarNode end, Obstacle obs){
	double theta = Math.atan((Position.WALL_CLEARANCE() + obs.getDiameter()) / start.getDistTo(obs));
	double leftBound = start.getAngleTurnTo(obs) - theta;
	double rightBound = start.getAngleTurnTo(obs) + theta;
	if(rightBound > Math.PI * 2) rightBound = rightBound - 2 * Math.PI;
	double angle = start.getAngleTurnTo(end);
	if(leftBound < rightBound){
	    if(angle > leftBound && angle < rightBound) return false;
	    else return true;
	}
	else{
	    if(angle > rightBound && angle < leftBound) return true;
	    else return false;
	}
    }
    
    private Path createPath(AStarNode end){
	Path path = new Path();
	AStarNode ptr = end;
	while(ptr != null){
	    path.addFirst(ptr);
	    ptr = ptr.getPrevious();
	}
	return path;
    }
    
    private void setAngles(Path path){
	boolean skipFirst = false;
	for(Position p : path){
	    if(!skipFirst)
		skipFirst = true;
	    else{
		((AStarNode)p).setAngle();
	    }
	}
    }
    
    private class AStarNode extends Position{
	private ArrayList<AStarNode> connected = new ArrayList<>(50);
	private AStarNode previous;
	private float distance;
	private float heruistic;
	private boolean visited = false;
	private boolean found = false;

	public AStarNode(float x_pos, float y_pos) {
	    super(x_pos, y_pos, 0.0, 0.0F);
	}
	
	public AStarNode(Position pos){
	    this(pos.getX(), pos.getY());
	}
	
	public void connect(AStarNode connect){
	    connected.add(connect);
	}
	
	public void resetConnection(){
	    connected = new ArrayList<>(50);
	}
	
	public AStarNode getPrevious(){
	    return previous;
	}
	
	public ArrayList<AStarNode> getConnections(){
	    return connected;
	}
	
	public float getDist(){
	    return distance;
	}
	
	public float getHeruistic(){
	    return heruistic;
	}
	
	public float getFScore(){
	    return getDist() + getHeruistic();
	}
	
	public AStarNode getMinFScore(){
	    AStarNode save = null;
	    for(AStarNode node : connected){
		if(save == null){
		    save = node;
		}
		else{
		    if(save.getFScore() > node.getFScore() && !node.isVisited())
			save = node;
		}
	    }
	    return save;
	}
	
	public boolean isVisited(){
	    return visited;
	}
	
	public boolean found(){
	    return found;
	}
	
	public void setPrevious(AStarNode previous){
	    this.previous = previous;
	}
	
	public void updateDist(float distance){
	    this.distance = distance;
	}
	
	public void setHeruistic(AStarNode end){
	    this.heruistic = this.getDistTo(end);
	}
	
	public void setAngle(){
	    super.setAngle(getPrevious().getAngleTurnTo(this) + getPrevious().getAngle());
	}
	
	public void setVisited(boolean visited){
	    this.visited = visited;
	}
	
	public void setFound(boolean found){
	    this.found = found;
	}
	
    }
}