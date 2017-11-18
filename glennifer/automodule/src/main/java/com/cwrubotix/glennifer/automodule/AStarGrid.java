package com.cwrubotix.glennifer.automodule;

import com.cwrubotix.glennifer.automodule.PathFindingAlgorithm;

public class AStarGrid implements PathFindingAlgorithm {
    public AStarGrid(float width, float height, float resolution) {
        
    }

    /**
     * The algorithm itself should be implemented here. Actually computes the path that should be taken and returns
     * a {@code Path} instance.
     *
     * @param startPosition
     * @param endPosition
     * @return
     */
    public Path computePath(Position startPosition, Position endPosition) {
        return null;
    }

    /**
     * Method that computes path when new obstacle was added
     *
     * @param currentPos  current position of robot fed
     * @param newObstacle new obstacle that was added
     * @return the new path created
     */
    public Path computePath(Position currentPos, Obstacle newObstacle) {
        return null;
    }
}
