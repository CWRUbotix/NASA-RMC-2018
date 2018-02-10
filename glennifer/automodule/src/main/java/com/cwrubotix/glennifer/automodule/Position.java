package main.java.com.cwrubotix.glennifer.automodule;

import java.util.Comparator;

/**
 * Data type that represents a location inside the arena.
 * If angle is negative, the instance represents horizontal line in the arena.
 *
 * @author Seohyun Jung
 */

public class Position implements Cloneable {

    private float x_pos;
    private float y_pos;
    
    private static final float WALL_CLEARANCE = 0.3F; //I set this to 30cm for now because I am scared of walls
    private static final float ARENA_WIDTH = 3.78F;  //+/- 1.39F From the middle (Tag is the origin)
    private static final float ARENA_HEIGHT = 7.38F;

    public Position(float x_pos, float y_pos) {
        this.x_pos = x_pos;
        this.y_pos = y_pos;
    }

    public float getX() {
        return x_pos;
    }

    public boolean setX(float x_pos) {
        //checking whether input is within the arena
        if (x_pos < (ARENA_WIDTH() / 2) - WALL_CLEARANCE() && x_pos > (ARENA_WIDTH() / -2) + WALL_CLEARANCE()) {
            this.x_pos = x_pos;
            return true;
        }
        return false;
    }

    public float getY() {
        return y_pos;
    }

    public boolean setY(float y_pos) {
        //checking whether input is within the arena
        if (y_pos > WALL_CLEARANCE() && y_pos < 7.38F - WALL_CLEARANCE()) {
            this.y_pos = y_pos;
            return true;
        }
        return false;
    }

    /**
     * Returns distance need to travel to position b
     *
     * @param b destination
     * @return distance need to travel to position b
     */
    public float getDistTo(Position b) {
        return (float) Math.sqrt(Math.pow(getX() - b.getX(), 2) + Math.pow(getY() - b.getY(), 2));
    }



    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Position) {
            Position compare = (Position) obj;
            float x_diff = compare.getX() - getX();
            float y_diff = compare.getY() - getY();
            if (Math.abs(x_diff) < 1e-5 && Math.abs(y_diff) < 1e-5)
                return true;
        }
        return false;
    }

    @Override
    public String toString() {
        return "(" + getX() + ", " + getY() + ")";
    }

    public static Comparator<Position> getComparatorByDistTo(final Position pos) {
        return new Comparator<Position>() {
            public int compare(Position a, Position b) {
                if (a.getDistTo(pos) < b.getDistTo(pos))
                    return -1;
                else if (a.getDistTo(pos) > b.getDistTo(pos))
                    return 1;
                else
                    return 0;
            }
        };
    }

    @Override
    public Object clone() {
        return new Position(getX(), getY());
    }

    /**
     * @return Feel free to come up with a better hashing algorithm.
     */
    @Override
    public int hashCode() {
        int hash = Float.floatToRawIntBits(getX()) ^ Float.floatToRawIntBits(getY());
        return hash;
    }

    public static final float WALL_CLEARANCE() {
        return Position.WALL_CLEARANCE;
    }

    public static final float ARENA_WIDTH() {
        return Position.ARENA_WIDTH;
    }

    public static final float ARENA_HEIGHT() {
        return Position.ARENA_HEIGHT;
    }
}
