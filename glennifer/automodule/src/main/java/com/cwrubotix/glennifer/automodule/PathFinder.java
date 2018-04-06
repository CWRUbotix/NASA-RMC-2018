package com.cwrubotix.glennifer.automodule;

/**
 * 
 * @author Robbie Dozier
 * @author Seohyun Jung
 * <p>
 * Unfinished
 * </p>
 */
public class PathFinder {
    private ModifiedAStar pathFindingAlgorithm;
    private Path path;
    private Position currentPos;
    private Position startPos;
    private Position targetPos;

    public PathFinder(ModifiedAStar pathFindingAlgorithm, Position startPos, Position targetPos) throws PathFindingAlgorithm.AlgorithmFailureException {
        this.pathFindingAlgorithm = pathFindingAlgorithm;
        this.startPos = startPos;
        this.targetPos = targetPos;
        path = pathFindingAlgorithm.computePath(startPos, targetPos);
        this.currentPos = startPos;
    }

    public PathFindingAlgorithm getAlgorithm() {
        return pathFindingAlgorithm;
    }

    public Path getPath() {
        return path;
    }

    public Position getCurrentPos() {
        return currentPos;
    }

    public void setCurrentPos(Position currentPos) {
        this.currentPos = currentPos;
    }

    public Position getStartPos() {
        return startPos;
    }

    public Position getTargetPos() {
        return targetPos;
    }

    public void runAlgorithm() {
        path = pathFindingAlgorithm.computePath();
    }

    public void registerObstacle(Obstacle obstacle) throws PathFindingAlgorithm.AlgorithmFailureException, DestinationModified{
        Path path = pathFindingAlgorithm.computePath(currentPos, obstacle);
        Position newDest = isDestReasonable(path);
        if(newDest == null){
            this.path = path;
        }
        else{
            pathFindingAlgorithm.setEnd(newDest);
            targetPos = newDest;
            this.path = pathFindingAlgorithm.computePath();
            throw new DestinationModified(newDest.getX(), newDest.getY());
        }
    }

    private Position isDestReasonable(Path path){
	float minX = 0.0F, maxX = 0.0F;
	
	for(Position pos : path){
	    if(pos.getX() < minX)
		minX = pos.getX();
	    if(pos.getX() > maxX)
		maxX = pos.getX();
	}
	
	if(Math.abs(maxX - getTargetPos().getX()) > Position.ARENA_WIDTH() * 1 / 3){
	    return new Position(maxX, getTargetPos().getY());
	}
	else if(Math.abs(minX - getTargetPos().getX()) > Position.ARENA_WIDTH() * 1 / 2){
	    return new Position(minX, getTargetPos().getY());
	}
	
	return null;
    }
    
    public class DestinationModified extends Exception{
	private float new_dest_x;
	private float new_dest_y;
	
	public DestinationModified(float new_dest_x, float new_dest_y){
	    super();
	    this.new_dest_x = new_dest_x;
	    this.new_dest_y = new_dest_y;
	}
	
	public float getX(){
	    return new_dest_x;
	}
	
	public float getY(){
	    return new_dest_y;
	}
    }
}
