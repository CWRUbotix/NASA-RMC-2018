package com.cwrubotix.glennifer.automodule;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Directional graph implementation
 *
 * @param <T> vertex value type (Graph cannot contain more than one vertex of the same value)
 * @param <G> graph edge weight type
 */
public class Graph<T, G extends Comparable<? super G>> {
    /** List of all vertices in graph */
    public HashMap<T, Vertex<T, G>> vertices;

    /**
     * Creates a graph with a single vertex
     *
     * @param baseVertexValue value of first vertex
     */
    public Graph(T baseVertexValue) {
        vertices = new HashMap<>();
        vertices.put(baseVertexValue, new Vertex<T, G>(baseVertexValue));
    }

    public void add(Vertex parent, Vertex child, G weight, boolean overwrite) throws IllegalArgumentException {
        if (!isInGraph(parent))
            throw new IllegalArgumentException("Parent vertex must be in the graph");

        if (overwrite && isInGraph(child))
            remove(child);
        else if (!overwrite && isInGraph(child))
            throw new IllegalArgumentException("Duplicate vertex value found:" + child + ", " + vertices.get(child) + " " + vertices);

        parent.connect(child, weight);
        vertices.put((T) child.getValue(), child);
    }

    public void connect(Vertex parent, Vertex child, G weight) throws IllegalArgumentException {
        if (!isInGraph(child))
            add(parent, child, weight, true);
        else {
            parent.connect(child, weight);
        }
    }

    public void disconnect(Vertex parent, Vertex child) throws IllegalArgumentException {
        if (!isInGraph(parent) || !isInGraph(child))
            throw new IllegalArgumentException("Vertices must be in the graph");

        parent.disconnect(child);
    }

    public void remove(Vertex vertex) {
        for (Vertex i : getVertices()) {
            if (i.getAdjacentVertices().contains(vertex))
                disconnect(i, vertex);
        }
        vertices.remove(vertex.getValue());
    }

    public Vertex get(T value) {
        for (T key : (T[]) vertices.keySet().toArray()) {
            if (key.equals(value))
                return vertices.get(key);
        }
        return null;
    }

    public ArrayList<Vertex<T, G>> getVertices() {
        return new ArrayList<>(vertices.values());
    }

    public boolean isInGraph(Vertex vertex) {
        return getVertices().contains(vertex);
    }


}
