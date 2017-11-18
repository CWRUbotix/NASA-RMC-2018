package main.java.com.cwrubotix.glennifer.automodule;

import main.java.com.cwrubotix.glennifer.automodule.PathFinder;
import main.java.com.cwrubotix.glennifer.automodule.Position;
import main.java.com.cwrubotix.glennifer.automodule.Path;

public class PathFinderDijkstra implements PathFindingAlgorithm {
    /** Position array of obstacles in arena */
    private Position[] obstacles;

    public PathFinderDijkstra(Position[] obstacles) {
        this.obstacles = obstacles;
    }

    /**
     * @return A {@code Position} array of the visible obstacles
     */
    public Position[] getObstacles() {
        return obstacles;
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

	@Override
	public Path computePath(Position currentPos, Obstacle newObstacle) {
		// TODO Auto-generated method stub
		return null;
	}
}
