package com.cwrubotix.glennifer.automodule;

/**
 * @author Robbie Dozier
 *
 * General interface for pathfinding algorithms to conform to. Utilizes the {@code Position} and {@code Path} modules
 * from the com.cwrubotix.glennifer.automodule package.
 */
public interface PathFinder {
    /**
     * The algorithm itself should be implemented here. Actually computes the path that should be taken and returns
     * a {@code Path} instance.
     *
     * @param startPosition Start of path (current position)
     * @param endPosition End of path (target position)
     * @return
     */
    public Path computePath(Position startPosition, Position endPosition);
}
