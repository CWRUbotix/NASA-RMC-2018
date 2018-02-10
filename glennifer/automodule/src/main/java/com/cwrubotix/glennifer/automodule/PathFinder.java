package main.java.com.cwrubotix.glennifer.automodule;


/**
 * @param <T> Path finding algorithm
 * @author Robbie Dozier
 * <p>
 * Unfinished
 */
public class PathFinder {
    private ModifiedAStar pathFindingAlgorithm;
    private Path path;
    private RobotPosition currentPos;
    private RobotPosition startPos;
    private RobotPosition targetPos;

    public PathFinder(ModifiedAStar pathFindingAlgorithm, RobotPosition startPos, RobotPosition targetPos) throws PathFindingAlgorithm.AlgorithmFailureException {
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

    public void setCurrentPos(RobotPosition currentPos) {
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

    public void registerObstacle(Obstacle obstacle) throws PathFindingAlgorithm.AlgorithmFailureException {
        path = pathFindingAlgorithm.computePath(currentPos, obstacle);
    }
}
