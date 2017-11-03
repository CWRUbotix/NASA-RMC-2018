package com.cwrubotix.glennifer.automodule;

public class PathFinderDijkstra implements PathFinder {
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
}
