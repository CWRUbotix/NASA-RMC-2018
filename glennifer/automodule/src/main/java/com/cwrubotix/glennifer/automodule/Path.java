package com.cwrubotix.glennifer.automodule;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class Path implements Iterable<Position> {
    private LinkedList<Position> path;

    public Path() {
        path = new LinkedList<Position>();
    }

    public Path(Position start, Position destination) {
        path = new LinkedList<Position>();
        path.add(start);
        path.add(destination);
    }

    /**
     * Hopefully this makes it so you can just pass in a list of positions and it'll work
     * -Robbie
     *
     * @param path
     */
    public Path(List<Position> path) {
        this.path = new LinkedList<>(path);
    }

    // Please don't turn this back to protected
    public LinkedList<Position> getPath() {
        return path;
    }

    /*
     * NOTE: this method returns path in array of Positions form
     * This method is for testing/debugging purpose only.
     * Use this to check whether your path generator method is working properly.
     * Should not be used for other purpose and will be removed once PathPlanSimulator is done.
     */
    public Position[] getList() {
        return (Position[]) getPath().toArray();
    }

    public Position getPoint(int index) {
        return path.get(index);
    }

    public int length() {
        return getPath().size();
    }

    public void add(int index, Position point) {
        getPath().add(index, point);
    }

    public void addFirst(Position point) {
        getPath().addFirst(point);
    }

    public void addLast(Position point) {
        getPath().addLast(point);
    }

    public Position remove(int index) {
        Position save = getPath().remove(index);
        return save;
    }

    public boolean remove(Position point) {
        if (getPath().remove(point)) {
            return true;
        }
        return false;
    }

    public Iterator<Position> iterator() {
        return getPath().iterator();
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Path) {
            Path compare = (Path) obj;
            return this.getPath().equals(compare.getPath());
        }
        return false;
    }
}
