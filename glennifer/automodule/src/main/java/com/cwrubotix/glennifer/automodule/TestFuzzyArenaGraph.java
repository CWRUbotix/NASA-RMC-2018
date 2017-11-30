package com.cwrubotix.glennifer.automodule;

import com.cwrubotix.glennifer.automodule.astargrid.FuzzyArenaGraph;

import java.util.Comparator;
import java.util.LinkedList;

public class TestFuzzyArenaGraph {
    public static void main(String[] args) {
        LinkedList<Integer> l = new LinkedList<>();
        l.add(3);
        l.add(5);
        l.add(3);
        l.add(2);
        l.add(0);
        l.add(10);
        l.add(1);

        l.sort(new Comparator<Integer>() {
            @Override
            public int compare(Integer o1, Integer o2) {
                if (o1 < o2)
                    return 1;
                else if (o1 > o2)
                    return -1;
                else return 0;
            }
        });

        System.out.println(l);
    }
}
