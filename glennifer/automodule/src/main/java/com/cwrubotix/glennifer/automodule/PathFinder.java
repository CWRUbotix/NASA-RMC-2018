package com.cwrubotix.glennifer.automodule;

import com.cwrubotix.glennifer.automodule.Path;
import com.cwrubotix.glennifer.automodule.Position;

import java.util.ArrayList;


/**
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
        pathFindingAlgorithm.computePath()
    }
}
