package com.cwrubotix.glennifer.automodule;

public class TestFuzzyArenaGraph {
    public static void main(String[] args) {
        FuzzyArenaGraph graph = new FuzzyArenaGraph(Position.ARENA_WIDTH(), Position.ARENA_HEIGHT(), 0.1);
        System.out.println(graph.getVertices());
    }
}
