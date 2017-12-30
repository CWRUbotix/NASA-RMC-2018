package main.java.com.cwrubotix.glennifer.automodule;

/**
 * @author Robbie Dozier
 *
 * General interface for pathfinding algorithms to conform to. Utilizes the {@code Position} and {@code Path} modules
 * from the main.java.com.cwrubotix.glennifer.automodule package.
 */
public interface PathFindingAlgorithm {
    /**
     * The algorithm itself should be implemented here. Actually computes the path that should be taken and returns
     * a {@code Path} instance. Returns null if no start or end position is specified.
     *
     * @return
     *
     * @throws AlgorithmFailureException when algorithm fails
     */
    public default Path computePath() throws AlgorithmFailureException{
	throw new UnsupportedOperationException();
    }

    /**
     * Call this method to update the start and end position
     *
     * @param startPosition
     * @param endPosition
     * @return
     * @throws AlgorithmFailureException when algorithm fails
     */
    public abstract Path computePath(Position startPosition, Position endPosition) throws AlgorithmFailureException;
    
    /**
     * Method that computes path when new obstacle was added
     * @param currentPos current position of robot fed (do we need this? --Robbie)
     * @param newObstacle new obstacle that was added
     * @return the new path created (FULL path, not just from currentPos to the obstacle)
     * @throws AlgorithmFailureException when algorithm fails
     */
    public abstract Path computePath(Position currentPos, Obstacle newObstacle) throws AlgorithmFailureException;

    public class AlgorithmFailureException extends RuntimeException {
    }
}
