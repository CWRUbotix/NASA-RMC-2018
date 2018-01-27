package com.cwrubotix.glennifer.automodule;

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
    /**
     * <p>
     * Represent the angle the robot is facing. EX) 0 when facing north, PI/2 when facing to the right and so forth (clock-wise).
     * </p>
     * <p>
     * Range : [0, 2PI) unit in radians.
     * </p>
     * If angle is negative, the position represents a horizontal line. (For dividing up arena purpose)
     */
    private double angle; //declared double 'cause java.lang.Math hates float angles for trigonometry.
    private float tilt;
    private static final float WALL_CLEARANCE = 0.3F; //I set this to 30cm for now because I am scared of walls
    private static final float ARENA_WIDTH = 3.78F;  //+/- 1.39F From the middle (Tag is the origin)
    private static final float ARENA_HEIGHT = 7.38F;

    public Position(float x_pos, float y_pos) {
        this(x_pos, y_pos, 0.0, 0.0F);
    }

    public Position(float x_pos, float y_pos, double angle, float tilt) {
        this.x_pos = x_pos;
        this.y_pos = y_pos;
        this.angle = angle;
        this.tilt = tilt;
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

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        //checking whether angle is within the range
        if (angle >= Math.PI * 2) {
            this.angle = angle - Math.PI * 2;
        } else {
            this.angle = angle;
        }
    }

    public float getTilt() {
        return tilt;
    }

    public void setTilt(float tilt) {
        this.tilt = tilt;
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

    /**
     * Returns angle the robot need to be in order to face position b
     *
     * @param b the destination
     * @return angle the robot need to be in order to face position b
     */
    public double getAngleTurnTo(Position b) {
        if (getAngle() >= 0 && b.getAngle() >= 0) {
            float x_diff = b.getX() - getX();
            float y_diff = b.getY() - getY();
            if (x_diff < 0) {
                return Math.PI + Math.PI / 2 - Math.atan((double) (y_diff / x_diff));
            } else {
                return Math.PI / 2 - Math.atan((double) (y_diff / x_diff));
            }
        }
        //Heading to vertically down (a.k.a. position b is straight horizontal line)
        return Math.PI;
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
        return new Position(getX(), getY(), getAngle(), getTilt());
    }

    /**
     * @return Feel free to come up with a better hashing algorithm.
     */
    @Override
    public int hashCode() {
        int hash = Float.floatToRawIntBits(getX()) ^ Float.floatToRawIntBits(getY()) ^ Float.floatToRawIntBits((float) getAngle()) ^ Float.floatToRawIntBits(getTilt());
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
