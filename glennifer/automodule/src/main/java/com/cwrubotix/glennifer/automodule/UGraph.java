package main.java.com.cwrubotix.glennifer.automodule;

public class UGraph<T, G extends Comparable<? super G>> extends Graph {

    public UGraph(T baseVertexValue) {
        super(baseVertexValue);
    }

    @Override
    public void add(Vertex parent, Vertex child, Comparable weight, boolean overwrite) throws IllegalArgumentException {
        super.add(parent, child, weight, overwrite);
        child.connect(parent, weight);
    }

    @Override
    public void connect(Vertex parent, Vertex child, Comparable weight) throws IllegalArgumentException {
        super.connect(parent, child, weight);
        child.connect(parent, weight);
    }

    @Override
    public void disconnect(Vertex parent, Vertex child) throws IllegalArgumentException {
        super.disconnect(parent, child);
        child.disconnect(parent);
    }
}
