package main.java.com.cwrubotix.glennifer.automodule;

import main.java.com.cwrubotix.glennifer.automodule.PathFindingAlgorithm;
import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.Position;

import java.util.ArrayList;


/**
 * @author Robbie Dozier
 *
 * Unfinished
 * @param <T> Path finding algorithm
 */
public class PathFinder<T extends PathFindingAlgorithm> {
    private T pathFindingAlgorithm;
    private ArrayList<Position> obstacles;
    private Path path;
    private Position currentPos;
    private Position startPos;
    private Position targetPos;

    public PathFinder(T pathFindingAlgorithm, Position startPos, Position targetPos) {
        obstacles = new ArrayList<Position>();
        this.pathFindingAlgorithm = pathFindingAlgorithm;
        this.startPos = startPos;
        this.targetPos = targetPos;
        path = pathFindingAlgorithm.computePath(this.startPos, this.targetPos);
        this.currentPos = path.getStart();
    }

    public Path getPath() {
        return path;
    }

    public Position getCurrentPos() {
        return currentPos;
    }

    public Position getStartPos() {
        return startPos;
    }

    public Position getTargetPos() {
        return targetPos;
    }

    public void registerObstacle(Position obstacle) {
        obstacles.add(obstacle);
        path = pathFindingAlgorithm.computePath(currentPos, targetPos);
    }
}
