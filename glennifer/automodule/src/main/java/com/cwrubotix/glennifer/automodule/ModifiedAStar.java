package main.java.com.cwrubotix.glennifer.automodule;

import java.util.ArrayList;

/**
 * Complete graph implementation of A* search algorithm with account for obstacles that are not visible before attaining certain proximity.
 *
 * @author Seohyun Jung
 */
public class ModifiedAStar implements PathFindingAlgorithm {

    /**
     * ArrayList storing obstacles found in the arena
     */
    private ArrayList<Obstacle> obstacles = new ArrayList<>(6);
    /**
     * ArrayList storing all nodes that are created during the run
     */
    private ArrayList<AStarNode> nodes = new ArrayList<>(75);
    /**
     * AStarNode that represents start point
     */
    private AStarNode start;
    /**
     * AStarNode that represents end point
     */
    private AStarNode end;

    /**
     * Returns the list of obstacles
     *
     * @return the list of obstacle
     */
    protected ArrayList<Obstacle> getObstacles() {
        return obstacles;
    }

    /**
     * Returns the list of nodes created during the run
     *
     * @return the list of nodes created during the run
     */
    protected ArrayList<AStarNode> getNodes() {
        return nodes;
    }
    
    protected void setStart(Position startPosition){
	start = new AStarNode(startPosition);
	start.setHeading(startPosition.getHeading());
	connectToAll(start);
	addNode(getNodes(), start);
    }
    
    protected void setEnd(Position endPosition){
	end = new AStarNode(endPosition);
	connectToAll(end);
	addNode(getNodes(), end);
    }

    @Override
    public Path computePath() {
        return aStar(start, end);
    }

    @Override
    public Path computePath(Position startPosition, Position endPosition) {

	/*Setting up fields and nodes*/
        start = new AStarNode(startPosition);
        start.setHeading(startPosition.getHeading());
        end = new AStarNode(endPosition);
        start.connect(end);
        end.connect(start);
        addNode(getNodes(), start);
        addNode(getNodes(), end);
	
	/*Calls aStar method*/
        Path result = aStar(start, end);
	
	/*Post processing*/
        setAngles(result);
        return result;
    }

    @Override
    public Path computePath(Position currentPos, Obstacle newObstacle) {
	setStart(currentPos);
        getObstacles().add(newObstacle);          //Adding obstacle to the list
        createNodes(newObstacle);                 //Creates new AStarNodes around the obstacle registered.

        connectToAll(); // Reconstructs the graph to make sure no edges run into obstacles.
		
        /*Calls aStar method*/
        Path result = aStar(start, end);
        
	/*Post processing*/
        setAngles(result);
        return result;
    }
    
    /*Helper methods for computePath(currentPos, newObstacle) method*/

    /**
     * Creates 6 nodes around the obstacle given
     *
     * @param obs
     */
    private void createNodes(Obstacle obs){
	
	for(int i = 0; i < 10; i++){
	    double angle = Math.PI * i / 5;
	    float clearance = 0.80F / 2 + obs.getRadius(); //Somehow algorithm works better with more clearance distance...?
	    float x_pos = (float)(obs.getX() + clearance * Math.cos(angle));
	    if(x_pos > Position.ARENA_WIDTH() / -2 + Position.WALL_CLEARANCE() && x_pos < Position.ARENA_WIDTH() / 2 - Position.WALL_CLEARANCE())
		addNode(getNodes(), new AStarNode(x_pos, (float)(obs.getY() + clearance * Math.sin(angle))));
	}
    }

    /**
     * Reconstructs the graph making sure that no edge has an obstacle in the way
     */
    private void connectToAll() {

        for (AStarNode node : getNodes()) {
            node.resetConnection();
            for (AStarNode connect : getNodes()) {
                if (isValid(node, connect)) {
                    node.connect(connect);
                }
            }
        }
    }
    
    private void connectToAll(AStarNode newNode){
	for(AStarNode node : getNodes()){
	    if(isValid(newNode, node)){
		newNode.connect(node);
		node.connect(newNode);
	    }
	}
    }

    /**
     * Checks whether the edge between the nodes given is valid.
     *
     * @param start start node
     * @param end   end node
     * @return true if there is no obstacle that is between the two nodes given
     */
    private boolean isValid(AStarNode start, AStarNode end) {
        boolean check = true;
        for (Obstacle obs : getObstacles()) {
            if (isOnTheWay(start, end, obs))
                check = false;
        }
        return check;
    }

    /**
     * Calculates and determines whether give obstacle is in the way between two AStarNodes given
     *
     * @param start the start node
     * @param end   the end node
     * @param obs   the Obstacle being evaluated
     * @return true if the obstacle is between two nodes with account for clearance for robot
     */

    private boolean isOnTheWay(AStarNode start, AStarNode end, Obstacle obs){
	/*Making sure whether obstacle even has a chance to be on the way*/
	double x_left_bound = Math.min(start.getX(), end.getX()) - obs.getRadius() - 0.80;
	double x_right_bound = Math.max(start.getX(), end.getY()) + obs.getRadius() + 0.80;
	double y_top_bound = Math.min(start.getY(), end.getY()) - obs.getRadius() - 0.80;
	double y_bottom_bound = Math.max(start.getY(), end.getY()) + obs.getRadius() + 0.80;
	
	/*If it is not within the range we should worry about, return false*/
	if(obs.getX() < x_left_bound || obs.getX() > x_right_bound || obs.getY() < y_top_bound || obs.getY() > y_bottom_bound){
	    return false;
	}
	

	/*Angle between the tangent line of clearance range that intersects start node position and the line between start node and center of Obstacle*/
        double theta = Math.atan((0.80F / 2 + obs.getRadius()) / Math.abs(start.getDistTo(obs)));
	
	/*Absolute angle positions of two tangent lines of clearance ranges that intersects start position*/
        double leftBound = start.getHeadingTo(obs) - theta;
        double rightBound = start.getHeadingTo(obs) + theta;

        if (rightBound > Math.PI * 2) rightBound = rightBound - 2 * Math.PI; // In case the angle bounds
        if (leftBound < 0) leftBound = leftBound + 2 * Math.PI;              // exceed the angle range

        double angle = start.getHeadingTo(end); // absolute angle position of end node relative to the start node

        if (leftBound < rightBound) { // Normal case
            if (angle > leftBound && angle < rightBound) return true;
            else return false;
        } else { // Special case, when either leftBound or rightBound value exceeded angle range
            if (angle > rightBound && angle < leftBound) return false;
            else return true;
        }

    }
    
    private void addNode(ArrayList<AStarNode> list, AStarNode node){
	if(!list.contains(node))
	    list.add(node);
    }
    
    /*Main A* implementation method*/

    /**
     * Creates path using AStar algorithm with given start and end points.
     *
     * @param start start point of the path
     * @param end   end point of the path
     * @return the path created
     */
    private Path aStar(AStarNode start, AStarNode end) {

	/*pre-search setup*/
        astarSetup(end);
        AStarNode current = start;
        current.updateDist(0.0F);
        ArrayList<AStarNode> openSet = new ArrayList<>(getNodes().size());
        addNode(openSet, start);
        start.setFound(true);

        while (!openSet.isEmpty()) { // While there are nodes to evaluate
            if (current.equals(end)) // When reached the destination
                return createPath(start, end);
            openSet.remove(current);  // Removes the node whose shortest distance from start position is determined
            current.setVisited(true); // marking the field that is added to closedSet
            
            for (AStarNode neighbor : current.getConnections()) { 
                if (!neighbor.isVisited() && !neighbor.found()) { // if it is not seen before, add to open list
                    addNode(openSet,neighbor);
                    neighbor.setFound(true);
                    neighbor.setPrevious(current);
                    neighbor.updateDist(current.getDist() + current.getDistTo(neighbor));
                }
                else if(!neighbor.isVisited()){ //If seen before, update cost.
                    float tempGScore = current.getDist() + current.getDistTo(neighbor);
                    if (neighbor.getDist() > tempGScore) {
                        neighbor.updateDist(tempGScore);
                        neighbor.setPrevious(current);
                    }
                }
            }
            current = getMinFScore(openSet); // setting next node as a node with minimum fScore.
        }
	
	/*If search ends without returning a path, there is no possible path.*/
        throw new PathFindingAlgorithm.AlgorithmFailureException();
    }
    
    /*Helper methods for astar(start, end) method*/

    /**
     * Returns the node with minimun fScore among the nodes in the given list
     *
     * @param openSet the given list to search
     * @return the node with minimum fScore among the nodes in the given list
     */
    private AStarNode getMinFScore(ArrayList<AStarNode> openSet) {
        AStarNode save = null;
        for (AStarNode neighbor : openSet) { //linear search
            if (save == null) {
                save = neighbor;
            } else {
                if (save.getFScore() > neighbor.getFScore())
                    save = neighbor;
            }
        }
        return save;
    }
    
    
    /*Helper methods for pre-setup for new search*/
    
    /**
     * Pre-search setup for astar method.
     * Resets field values to default and sets up heuristic of all nodes
     * @param end destination of the search
     */
    private void astarSetup(AStarNode end){
	for(AStarNode node : getNodes()){
            node.setHeruistic(end);
            node.setVisited(false);
            node.setFound(false);
            node.updateDist(Float.POSITIVE_INFINITY);
	}
    }
    
    /*Post-search processing*/

    /**
     * Called after path search to construct path with given destination
     *
     * @param start the start node
     * @param end   the destination node
     * @return the result path created
     */
    private Path createPath(AStarNode start, AStarNode end) {
        Path path = new Path();
        AStarNode ptr = end;
        while (!ptr.equals(start)) {
            path.addFirst(ptr);
            ptr = ptr.getPrevious();
        }
        path.addFirst(start);

        return path;
    }

    /**
     * Iterates through nodes in the given path and sets up angle fields
     *
     * @param path the path whose nodes needs their angle field set up.
     */
    private void setAngles(Path path) {
        boolean skipFirst = false;
        for (Position p : path) {
            if (!skipFirst)
                skipFirst = true;
            else {
                ((AStarNode) p).setHeading();
            }
        }
    }

    /**
     * Class that represents the vertex of the graph in A* search
     *
     * @author Seohyun Jung
     * @see Position
     */
    private class AStarNode extends Position {

        /**
         * list that store neighboring vertexes
         */
        private ArrayList<AStarNode> connected = new ArrayList<>(50);
        /**
         * the vertex to reach this vertex with shortest distance
         */
        private AStarNode previous;
        /**
         * the total distance from start point to this vertex
         */
        private float distance;
        /**
         * the heruistic value of this vertex
         */
        private float heruistic;
        /**
         * indicator of whether this node belongs to closed set during each A* search
         */
        private boolean visited = false;
        /**
         * indicator of whether this node belongs to open set during each A* search
         */
        private boolean found = false;

        /**
         * Creates AStarNode with given coordinate positions
         *
         * @param x_pos x-coordinate position of the vertex
         * @param y_pos y-coordinate position of the vertex
         */
        public AStarNode(float x_pos, float y_pos) {
            super(x_pos, y_pos);
        }

        /**
         * Creates AStarNode with given position
         *
         * @param pos the position of the vertex
         */
        public AStarNode(Position pos) {
            this(pos.getX(), pos.getY());
        }

        /**
         * Adds given AStarNode to the neighboring list
         *
         * @param connect the vertex to add to the neighboring list
         */
        public void connect(AStarNode connect) {
            connected.add(connect);
        }

        /**
         * Resets the neighboring list
         */
        public void resetConnection() {
            connected = new ArrayList<>(50);
        }
	
	/*Getter methods*/

        /**
         * Returns the previous node to reach this node with shortest distance from the start node
         *
         * @return the previous node to reach this node with shortest distance from the start node
         */
        public AStarNode getPrevious() {
            return previous;
        }

        /**
         * Returns list of neighboring nodes
         *
         * @return list of neighboring nodes
         */
        public ArrayList<AStarNode> getConnections() {
            return connected;
        }

        /**
         * Returns the known shortest distance from the start node to this node
         *
         * @return the known shortest distance from the start node to this node
         */
        public float getDist() {
            return distance;
        }

        /**
         * Returns the heruistic value of the node
         *
         * @return the heruistic value of the node
         */
        public float getHeruistic() {
            return heruistic;
        }

        /**
         * Returns the fscore of this node. fscore is calculated by summing up the known shortest distance from the start node and heruistic value together
         *
         * @return the fscore of this node
         */
        public float getFScore() {
            return getDist() + getHeruistic();
        }

        /**
         * Returns whether this node belongs to closed set during A* search
         *
         * @return whether this node belongs to closed set during A* search
         */
        public boolean isVisited() {
            return visited;
        }

        /**
         * Returns whether this node belongs to open set during A* search
         *
         * @return whether this node belongs to open set during A* search
         */
        public boolean found() {
            return found;
        }
	
	/*Setter methods*/

        /**
         * Sets previous field with given AStarNode
         *
         * @param previous the node that this node came from to attain shortest distance from the start node
         */
        public void setPrevious(AStarNode previous) {
            this.previous = previous;
        }

        /**
         * Sets the distance field with given distance
         *
         * @param distance the known shortest distance from the start node
         */
        public void updateDist(float distance) {
            this.distance = distance;
        }

        /**
         * Calculates and updates new heruistic value with given destination position
         *
         * @param end the destination of the path search
         */
        public void setHeruistic(AStarNode end) {
            this.heruistic = this.getDistTo(end);
        }

        /**
         * Calculates and updates angle value using the previous AStarNode
         */
        public void setHeading() {
            super.setHeading(getPrevious().getHeadingTo(this)); //The absolute angle position that robot needs to face to reach this position
        }

        /**
         * Marks whether this node has been added to closed set during A* search
         *
         * @param visited
         */
        public void setVisited(boolean visited) {
            this.visited = visited;
        }

        /**
         * Marks whether this node has been added to open set during A* search
         *
         * @param found
         */
        public void setFound(boolean found) {
            this.found = found;
        }
    }
}