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

    public PathFinder() {
        obstacles = new ArrayList<Position>();
    }

    public Path getPath() {

    }

    public void registerObstacle(Position obstacle) {
        obstacles.add(obstacle);
        // pathFindingAlgorithm.computePath();
    }
}
