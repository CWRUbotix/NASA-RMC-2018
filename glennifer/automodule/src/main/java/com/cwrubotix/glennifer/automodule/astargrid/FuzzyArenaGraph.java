package com.cwrubotix.glennifer.automodule.astargrid;

import com.cwrubotix.glennifer.automodule.FuzzyPosition;
import com.cwrubotix.glennifer.automodule.Graph;
import com.cwrubotix.glennifer.automodule.Vertex;
import com.cwrubotix.glennifer.automodule.Origin;
import com.cwrubotix.glennifer.automodule.UGraph;

import java.util.ArrayList;

public class FuzzyArenaGraph extends UGraph<FuzzyPosition, Double> {

    public FuzzyArenaGraph(double width, double height, double error, double resolution) throws IllegalArgumentException {
        // Sets origin as base node
        super(new FuzzyPosition(0.0F, 0.0F, error));

        if (resolution <= error)
            throw new IllegalArgumentException("resolution must be greater than error");

        // adds nodes in positive x direction
        Vertex lastPos = null;
        for (float i = 0.0F; i < width / 2; i += resolution) {
            if (lastPos == null)
                lastPos = this.get(new Origin());
            Vertex newPos = new Vertex(new FuzzyPosition(i, 0.0F, error));
            this.add(lastPos, newPos, resolution, false);
            lastPos = newPos;
        }
        // adds nodes in negative x direction
        lastPos = null;
        for (float i = 0.0F; i > -1 * width / 2; i -= resolution) {
            if (lastPos == null)
                lastPos = this.get(new Origin());
            Vertex newPos = new Vertex(new FuzzyPosition(i, 0.0F, error));
            this.add(lastPos, newPos, resolution, false);
            lastPos = newPos;
        }
        // adds nodes in positive y direction
        for (Object o : this.getVertices().toArray()) {
            Vertex xPos = (Vertex) o;
            for (float i = 0.0F; i < height; i += resolution) {
                if (lastPos == null)
                    lastPos = this.get(new Origin());
                Vertex newPos = new Vertex(
                        new FuzzyPosition(((FuzzyPosition) xPos.getValue()).getX(), i, error));
                this.add(lastPos, newPos, resolution, false);
                lastPos = newPos;
            }
        }
        // connect adjacent nodes
        for (Vertex vertex : (ArrayList<Vertex>) this.getVertices()) {
            FuzzyPosition position = (FuzzyPosition) vertex.getValue();
            // connect North
            Vertex north =
                    this.get(new FuzzyPosition(position.getX(), position.getY() + (float) resolution, error));
            if (north != null)
                this.connect(vertex, north, resolution);
            // connect East
            Vertex east =
                    this.get(new FuzzyPosition(position.getX() + (float) resolution, position.getY(), error));
            if (east != null)
                this.connect(vertex, east, resolution);
            // connect South
            Vertex south =
                    this.get(new FuzzyPosition(position.getX(), position.getY() - (float) resolution, error));
            if (south != null)
                this.connect(vertex, south, resolution);
            // connect West
            Vertex west =
                    this.get(new FuzzyPosition(position.getX() - (float) resolution, position.getY(), error));
            if (west != null)
                this.connect(vertex, west, resolution);
        }
    }
}
