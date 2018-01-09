package main.java.com.cwrubotix.glennifer.automodule.astargrid;

import main.java.com.cwrubotix.glennifer.automodule.*;

import java.util.*;

public class AStarGrid implements PathFindingAlgorithm {
    private FuzzyArenaGraph grid;
    private HashMap<Vertex, Double> heuristic;

    private Position startPosition;
    private Position endPosition;
    private Path currentPath;

    public AStarGrid(FuzzyArenaGraph grid) {
        this.grid = grid;
    }

    public AStarGrid(double error) {
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
        Vertex start = grid.get(startPosition); // Starting vertex
        Vertex end = grid.get(endPosition); // Target vertex
        HashMap<Vertex, Vertex> pathVertexList = new HashMap<>(); // Maps each vertex to the PREVIOUS one in the path
        LinkedList<Vertex<FuzzyPosition, Double>> openSet = new LinkedList<>(); // Set of unevaluated vertices
        ArrayList<Vertex<FuzzyPosition, Double>> closedSet = new ArrayList<>(); // Set of evaluated nodes
        HashMap<Vertex, Double> distanceList = new HashMap<>(); // LUT of total costs, initially Double.MAX_VALUE (infinity)

        // Fill distanceList with max values (infinity)
        for (Vertex<FuzzyPosition, Double> vertex : (ArrayList<Vertex>) grid.getVertices()) {
            distanceList.put(vertex, Double.MAX_VALUE);
        }
        distanceList.put(start, heuristic.get(start));

        // Astar Algorithm
        openSet.add(start);

        while (!openSet.isEmpty()) {
            // Sort openSet
            openSet.sort(Comparator.comparingDouble((Vertex<FuzzyPosition, Double> o)
                    -> distanceList.get(o) + heuristic.get(o)));

            // Pick most promising vertex (first in openSet)
            Vertex current = openSet.getFirst();

            // See if we've arrived at the end
            if (current == end) {
                LinkedList<Position> positionList = new LinkedList<>();
                Vertex next = current;
                while (next != null) {
                    positionList.add((Position) next.getValue());
                    next = pathVertexList.remove(next);
                }
                Collections.reverse(positionList);
                return currentPath = new Path(positionList);
            }

            // Transfer current from openSet to closedSet
            openSet.remove(current);
            closedSet.add(current);

            // Evaluate neighbors of current
            for (Vertex<FuzzyPosition, Double> neighbor :
                    (ArrayList<Vertex<FuzzyPosition, Double>>) current.getAdjacentVertices()) {
                // If the neighbor is already in closedSet, skip it
                if (!closedSet.contains(neighbor)) {
                    // If the neighbor isn't in openSet, add it
                    if (!openSet.contains(neighbor))
                        openSet.add(neighbor);

                    // Compute cost of neighbor
                    double tentativeCost = (Double) current.getWeightFor(neighbor) + distanceList.get(current);
                    // If it costs less to get to neighbor than the current value in distanceList, then update it
                    double currentCost = distanceList.get(neighbor);
                    if (tentativeCost < currentCost) {
                        pathVertexList.put(neighbor, current);
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
        System.out.println("New obstacle " + newObstacle);
        for (Vertex<FuzzyPosition, Double> vertex : (ArrayList<Vertex<FuzzyPosition, Double>>) grid.getVertices()) {
            // if (newObstacle.equals(vertex.getValue()))
            if (newObstacle.getDistTo(vertex.getValue()) <= newObstacle.getRadius() + vertex.getValue().getError() * Math.sqrt(2.) + 0.8) {
                System.out.println("Rerouting to avoid position " + vertex.getValue());
                grid.remove(vertex);
            }
        }
        Path oldPath = currentPath;
        computePath(currentPos, endPosition);
        LinkedList<Position> newList = new LinkedList<>();
        boolean found = false;
        for (int i = 0; i < oldPath.length(); i++) {
            if (currentPos.equals(oldPath.getPath().get(i)))
                found = true;
            if (!found)
                newList.add(oldPath.getPath().get(i));
        }
        newList.addAll(currentPath.getPath());
        System.out.println("Path from " + newList.getFirst() + " to " + newList.getLast());
        return currentPath = new Path(newList);
    }

    private void computeHeuristic() {
        heuristic = new HashMap<>();
        for (Vertex<FuzzyPosition, Double> vertex : (ArrayList<Vertex>) grid.getVertices()) {
            heuristic.put(vertex, (double) vertex.getValue().getDistTo(endPosition));
        }
    }
}
