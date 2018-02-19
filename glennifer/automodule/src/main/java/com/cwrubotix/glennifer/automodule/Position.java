package com.cwrubotix.glennifer.automodule;

/**
 * Data type that represents a location inside the arena.
 *
 * @author Seohyun Jung
 */

public class Position extends Coordinate implements Cloneable {

    /**
     * <p>
     * Represent the heading the robot is facing. EX) 0 when facing north, PI/2 when facing to the right and so forth (clock-wise).
     * </p>
     * <p>
     * Range : [0, 2PI) unit in radians.
     * </p>
     */
    private double heading; //declared double 'cause java.lang.Math hates float headings for trigonometry.
    


    public Position(float x_pos, float y_pos, double heading) {
        super(x_pos, y_pos);
        this.heading = heading;
    }
    
    public Position(float x_pos, float y_pos){
	this(x_pos, y_pos, 0.0);
    }
    
    public double getHeading(){
	return heading;
    }
    
    public void setHeading(double heading){
	this.heading = heading;
    }
    
    public boolean setX(float x_pos) {
        //checking whether input is within the arena
        if (x_pos < (ARENA_WIDTH() / 2) - WALL_CLEARANCE() && x_pos > (ARENA_WIDTH() / -2) + WALL_CLEARANCE()) {
            super.setX(x_pos);
            return true;
        }
        return false;
    }
    
    public boolean setY(float y_pos) {
        //checking whether input is within the arena
        if (y_pos > WALL_CLEARANCE() && y_pos < ARENA_HEIGHT() - WALL_CLEARANCE()) {
            super.setY(y_pos);
            return true;
        }
        return false;
    }
    
    /**
     * Returns heading the robot need to be in order to face position b
     *
     * @param p the destination
     * @return angle the robot need to be in order to face position b
     */
    public double getHeadingTo(Coordinate p){
   	float x_diff = p.getX() - getX();
   	float y_diff = p.getY() - getY();
   	if (x_diff < 0) {
   	    return Math.PI + Math.PI / 2 - Math.atan((double) (y_diff / x_diff));
   	} else {
   	    return Math.PI / 2 - Math.atan((double) (y_diff / x_diff));
   	}
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


    @Override
    public Object clone() {
        return new Position(getX(), getY(), getHeading());
    }

    /**
     * @return Feel free to come up with a better hashing algorithm.
     */
    @Override
    public int hashCode() {
        int hash = Float.floatToRawIntBits(getX()) ^ Float.floatToRawIntBits(getY());
        return hash;
    }

    
}
