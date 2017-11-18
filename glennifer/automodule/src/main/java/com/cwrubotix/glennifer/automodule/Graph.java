package main.java.com.cwrubotix.glennifer.automodule;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Directional graph implementation
 *
 * @param <T> vertex value type (Graph cannot contain more than one vertex of the same value)
 * @param <G> graph edge weight type
 */
public class Graph<T, G extends Comparable> {
    /** List of all vertices in graph */
    private HashMap<T, Vertex> vertices;

    /**
     * Creates a graph with a single vertex
     *
     * @param baseVertexValue value of first vertex
     */
    public Graph(T baseVertexValue) {
        vertices = new HashMap<>();
        vertices.put(baseVertexValue, new Vertex(baseVertexValue));
    }

    public void add(Vertex parent, Vertex child, G weight, boolean overwrite) throws IllegalArgumentException {
        if (!isInGraph(parent))
            throw new IllegalArgumentException("Parent vertex must be in the graph");

        if (overwrite && isInGraph(child))
            remove(child);
        else if (!overwrite)
            throw new IllegalArgumentException("Duplicate vertex value found");

        parent.connect(child, weight);
        vertices.put(child.getValue(), child);
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
        return vertices.get(value);
    }

    public ArrayList<Vertex> getVertices() {
        return new ArrayList<>(vertices.values());
    }

    public boolean isInGraph(Vertex vertex) {
        return getVertices().contains(vertex);
    }

    class Vertex {
        /** Value stored in the vertex (location, robot state, etc.) */
        private T value;
        /** Edges connected to this vertex in the form ({@code Vertex}, weight) */
        private HashMap<Vertex, G> edges;

        /**
         * Instantiates a {code Vertex} instance.
         *
         * @param value Value this vertex will have
         */
        public Vertex(T value) {
            this.value = value;
            this.edges = new HashMap<Vertex, G>();
        }

        /**
         * Instantiates a {@code Vertex} instance from an existing {@code Vertex}.
         *
         * @param value  Value this vertex will have
         * @param host   Host vertex
         * @param weight Weight from host vertex to this vertex. (Undirectedness is not implied)
         */
        public Vertex(T value, Vertex host, G weight) {
            this(value);
            host.connect(this, weight);
        }

        /**
         * @return Vertex value
         */
        public T getValue() {
            return this.value;
        }

        /**
         * Connects this vertex to another vertex.
         *
         * @param vertex Vertex to add to
         * @param weight Connection weight
         */
        public void connect(Vertex vertex, G weight) {
            edges.put(vertex, weight);
        }

        /**
         * Removes this vertex from another vertex.
         *
         * @param vertex Vertex from which to disconnect
         */
        public void disconnect(Vertex vertex) {
            edges.remove(vertex);
        }

        /**
         * @return ArrayList of adjecent {@code Vertex} instances
         */
        public ArrayList<Vertex> getAdjacentVertices() {
            return new ArrayList<>(edges.keySet());
        }

        public int getDegree() {
            return getAdjacentVertices().size();
        }

        /**
         * @param vertex adjacent {@code Vertex} object.
         * @return The weight for that vertex. {@code null} if that vertex isn't adjecent.
         */
        public G getWeightFor(Vertex vertex) {
            return edges.get(vertex);
        }
    }
}
