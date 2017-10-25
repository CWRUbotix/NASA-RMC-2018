package com.cwrubotix.glennifer.automodule;

import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.Position;

/**
 * @author Robbie Dozier
 *
 * General interface for pathfinding algorithms to conform to. Utilizes the {@code Position} and {@code Path} modules
 * from the com.cwrubotix.glennifer.automodule package.
 */
public interface PathFindingAlgorithm {
    /**
     * The algorithm itself should be implemented here. Actually computes the path that should be taken and returns
     * a {@code Path} instance.
     *
     * @param startPosition
     * @param endPosition
     * @return
     */
    public Path computePath(Position startPosition, Position endPosition);
}
