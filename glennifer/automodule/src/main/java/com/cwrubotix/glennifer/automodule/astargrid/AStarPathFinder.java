package main.java.com.cwrubotix.glennifer.automodule.astargrid;

import main.java.com.cwrubotix.glennifer.automodule.PathFinder;
import main.java.com.cwrubotix.glennifer.automodule.PathFindingAlgorithm;
import main.java.com.cwrubotix.glennifer.automodule.Position;

public class AStarPathFinder extends PathFinder {
    public AStarPathFinder(Position startPos, Position targetPos, double error) throws PathFindingAlgorithm.AlgorithmFailureException {
        super(new AStarGrid(error), startPos, targetPos);
    }
}
