package com.cwrubotix.glennifer.automodule;

import java.util.ArrayList;
import java.util.HashMap;

public class Vertex<T, G extends Comparable<? super G>> {
    /**
     * Value stored in the vertex (location, robot state, etc.)
     */
    private T value;
    /**
     * Edges connected to this vertex in the form ({@code Vertex}, weight)
     */
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

    @Override
    public String toString() {
        return getValue().toString();
    }

    @Override
    public int hashCode() {
        return getValue().hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Vertex) {
            Vertex compare = (Vertex) obj;
            return this.getValue().equals(((Vertex) obj).getValue());
        }
        return false;
    }
}
