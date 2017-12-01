package com.cwrubotix.glennifer.automodule.astargrid;

import com.cwrubotix.glennifer.automodule.*;
import com.sun.org.apache.xpath.internal.operations.Bool;

import java.util.*;
import java.util.function.ToDoubleFunction;

public class AStarGrid implements PathFindingAlgorithm {
    private FuzzyArenaGraph grid;
    private HashMap<Vertex, Double> heuristic;

    private Position startPosition;
    private Position endPosition;
    private Path currentPath;

    public AStarGrid(FuzzyArenaGraph grid) {
        this.grid = grid;
    }

    public AStarGrid(double error, double resolution) {
        grid = new FuzzyArenaGraph(Position.ARENA_WIDTH(), Position.ARENA_HEIGHT(), error);
    }

    /**
     * The algorithm itself should be implemented here. Actually computes the path that should be taken and returns
     * a {@code Path} instance.  Returns null if no start or end position is specified.
     *
     * @return
     */
    public Path computePath() throws PathFindingAlgorithm.AlgorithmFailureException {
        if (startPosition == null || endPosition == null)
            return null;

        if (heuristic == null)
            computeHeuristic();

        // Set up variables
        Vertex start = grid.get(startPosition);
        Vertex end = grid.get(endPosition);
        HashMap<Vertex, Vertex> pathVertexList = new HashMap<>(); // Maps each vertex to the next one in the path
        LinkedList<Vertex<FuzzyPosition, Double>> openSet = new LinkedList<>();
        ArrayList<Vertex<FuzzyPosition, Double>> closedSet = new ArrayList<>();
        HashMap<Vertex, Double> distanceList = new HashMap<>();

        for (Vertex<FuzzyPosition, Double> vertex : (ArrayList<Vertex>) grid.getVertices()) {
            distanceList.put(vertex, Double.NaN);
        }
        distanceList.put(start, 0.0);

        // Algorithm
        Vertex currentNode = start;
        openSet.add(start);

        while (!openSet.isEmpty()) {
            // Sort openSet
            openSet.sort(Comparator.comparingDouble((Vertex<FuzzyPosition, Double> o)
                    -> distanceList.get(o) + heuristic.get(o)));

            Vertex current = openSet.getFirst();

            if (current == end) {
                LinkedList<Position> positionList = new LinkedList<>();
                Vertex next = start;
                while (next != null) {
                    positionList.add((Position) next.getValue());
                    next = pathVertexList.remove(next);
                }
                positionList.add((Position) current.getValue());
                return currentPath = new Path(positionList);
            }

            openSet.remove(current);
            closedSet.add(current);

            for (Vertex<FuzzyPosition, Double> neighbor : (ArrayList<Vertex<FuzzyPosition, Double>>) current.getAdjacentVertices()) {
                if (!closedSet.contains(neighbor)) {
                    if (!openSet.contains(neighbor))
                        openSet.add(neighbor);

                    double tentativeCost = distanceList.get(current) + (Double) current.getWeightFor(neighbor);
                    if (distanceList.get(neighbor) == null || tentativeCost < distanceList.get(neighbor)) {
                        pathVertexList.put(current, neighbor);
                        distanceList.put(neighbor, tentativeCost);
                    }
                }
            }
        }
        throw new AlgorithmFailureException();
    }

    /**
     * Call this method to update the start and end position
     *
     * @param startPosition
     * @param endPosition
     * @return
     */
    public Path computePath(Position startPosition, Position endPosition) throws AlgorithmFailureException {
        this.startPosition = startPosition;
        this.endPosition = endPosition;
        computeHeuristic();
        return computePath();
    }

    /**
     * Method that computes path when new obstacle was added
     *
     * @param currentPos  current position of robot fed
     * @param newObstacle new obstacle that was added
     * @return the new path created
     */
    public Path computePath(Position currentPos, Obstacle newObstacle) throws AlgorithmFailureException {
        for (Vertex<FuzzyPosition, Double> vertex : (ArrayList<Vertex<FuzzyPosition, Double>>) grid.getVertices()) {
            if (newObstacle.equals(vertex.getValue()))
                grid.remove(vertex);
        }
        Path newPath = computePath(currentPos, endPosition);
        LinkedList<Position> newList = new LinkedList<>();
        int i = 0;
        while (currentPath.getList()[i] != currentPos)
            newList.add(currentPath.getList()[i]);
        newList.addAll(Arrays.asList(newPath.getList()));
        return currentPath = new Path(newList);
    }

    private void computeHeuristic() {
        heuristic = new HashMap<>();
        for (Vertex<FuzzyPosition, Double> vertex : (ArrayList<Vertex>) grid.getVertices()) {
            heuristic.put(vertex, (double) vertex.getValue().getDistTo(endPosition));
        }
    }
}
