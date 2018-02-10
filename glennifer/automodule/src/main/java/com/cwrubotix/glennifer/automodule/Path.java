package main.java.com.cwrubotix.glennifer.automodule;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class Path implements Iterable<RobotPosition> {
    private LinkedList<RobotPosition> path;

    public Path() {
        path = new LinkedList<RobotPosition>();
    }

    public Path(RobotPosition start, RobotPosition destination) {
        path = new LinkedList<RobotPosition>();
        path.add(start);
        path.add(destination);
    }

    /**
     * Hopefully this makes it so you can just pass in a list of positions and it'll work
     * -Robbie
     *
     * @param path
     */
    public Path(List<RobotPosition> path) {
        this.path = new LinkedList<>(path);
    }

    // Please don't turn this back to protected
    public LinkedList<RobotPosition> getPath() {
        return path;
    }

    /*
     * NOTE: this method returns path in array of Positions form
     * This method is for testing/debugging purpose only.
     * Use this to check whether your path generator method is working properly.
     * Should not be used for other purpose and will be removed once PathPlanSimulator is done.
     */
    public RobotPosition[] getList() {
        return (RobotPosition[]) getPath().toArray();
    }

    public RobotPosition getPoint(int index) {
        return path.get(index);
    }

    public int length() {
        return getPath().size();
    }

    public void add(int index, RobotPosition point) {
        getPath().add(index, point);
    }

    public void addFirst(RobotPosition point) {
        getPath().addFirst(point);
    }

    public void addLast(RobotPosition point) {
        getPath().addLast(point);
    }

    public RobotPosition remove(int index) {
        RobotPosition save = getPath().remove(index);
        return save;
    }

    public boolean remove(RobotPosition point) {
        if (getPath().remove(point)) {
            return true;
        }
        return false;
    }

    public Iterator<RobotPosition> iterator() {
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
