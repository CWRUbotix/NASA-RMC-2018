package main.java.com.cwrubotix.glennifer.automodule.astargrid;

import main.java.com.cwrubotix.glennifer.automodule.FuzzyPosition;
import main.java.com.cwrubotix.glennifer.automodule.Origin;
import main.java.com.cwrubotix.glennifer.automodule.UGraph;
import main.java.com.cwrubotix.glennifer.automodule.Vertex;

import java.util.ArrayList;

public class FuzzyArenaGraph extends UGraph<FuzzyPosition, Double> {
    public FuzzyArenaGraph(double width, double height, double error) {
        // Sets origin as base node
        super(new FuzzyPosition(0.0F, 0.0F, error));

        double delta = error * 2;

        // adds nodes in positive x direction
        Vertex lastPos = null;
        for (float i = (float) delta; i <= (width / 2); i += delta) {
            if (lastPos == null)
                lastPos = this.get(new Origin());
            Vertex newPos = new Vertex(new FuzzyPosition(i, 0.0F, error));
            this.add(lastPos, newPos, delta, false);
            lastPos = newPos;
        }
        // adds nodes in negative x direction
        lastPos = null;
        for (float i = -1 * (float) delta; i >= (-1 * width / 2); i -= delta) {
            if (lastPos == null)
                lastPos = this.get(new Origin());
            Vertex newPos = new Vertex(new FuzzyPosition(i, 0.0F, error));
            this.add(lastPos, newPos, delta, false);
            lastPos = newPos;
        }
        // adds nodes in positive y direction
        lastPos = null;
        for (Object o : this.getVertices().toArray()) {
            Vertex xPos = (Vertex) o;
            for (float i = (float) delta; i < height; i += delta) {
                if (lastPos == null)
                    lastPos = xPos;
                Vertex newPos = new Vertex(
                        new FuzzyPosition(((FuzzyPosition) xPos.getValue()).getX(), i, error));
                this.add(lastPos, newPos, delta, false);
                lastPos = newPos;
            }
        }

        // connect adjacent nodes
        for (Vertex vertex : (ArrayList<Vertex>) this.getVertices()) {
            FuzzyPosition position = (FuzzyPosition) vertex.getValue();
            // connect North
            Vertex north =
                    this.get(new FuzzyPosition(position.getX(), position.getY() + (float) delta, error));
            if (north != null)
                this.connect(vertex, north, delta);
            // connect East
            Vertex east =
                    this.get(new FuzzyPosition(position.getX() + (float) delta, position.getY(), error));
            if (east != null)
                this.connect(vertex, east, delta);
            // connect South
            Vertex south =
                    this.get(new FuzzyPosition(position.getX(), position.getY() - (float) delta, error));
            if (south != null)
                this.connect(vertex, south, delta);
            // connect West
            Vertex west =
                    this.get(new FuzzyPosition(position.getX() - (float) delta, position.getY(), error));
            if (west != null)
                this.connect(vertex, west, delta);
        }
    }
}
