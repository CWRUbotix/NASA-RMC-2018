package com.cwrubotix.glennifer.automodule.astargrid;

import com.cwrubotix.glennifer.automodule.Obstacle;
import com.cwrubotix.glennifer.automodule.PathFinder;
import com.cwrubotix.glennifer.automodule.PathFindingAlgorithm;
import com.cwrubotix.glennifer.automodule.Position;

public class AStarPathFinder extends PathFinder {
    public AStarPathFinder(Position startPos, Position targetPos, double error, double resolution) throws PathFindingAlgorithm.AlgorithmFailureException {
        super(new AStarGrid(error, resolution), startPos, targetPos);
    }
}
