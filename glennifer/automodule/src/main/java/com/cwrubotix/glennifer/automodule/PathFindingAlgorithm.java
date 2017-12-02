package main.java.com.cwrubotix.glennifer.automodule;

import main.java.com.cwrubotix.glennifer.automodule.Path;
import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.Obstacle;

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
    public abstract Path computePath(Position startPosition, Position endPosition);
    
    /**
     * Method that computes path when new obstacle was added
     * @param currentPos current position of robot fed
     * @param newObstacle new obstacle that was added
     * @return the new path created
     */
    public abstract Path computePath(Position currentPos, Obstacle newObstacle);
}
