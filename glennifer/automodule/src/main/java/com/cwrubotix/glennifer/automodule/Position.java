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
    private Obstacle nearestObs;


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
    
    /**
     * Returns the obstacle within the arena that is the nearest one from this node
     * 
     * @return the obstacle within the arena that is the nearest one from this node
     */
    public Obstacle getNearestObs(){
        return nearestObs;
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
     * Sets the nearest obstacle from this node as given input
     * 
     * @param obs the nearest obstacle from this node
     */
    public void setNearestObs(Obstacle obs){
        this.nearestObs = obs;
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
            return Math.abs(x_diff) < 1e-5 && Math.abs(y_diff) < 1e-5;
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", getX(), getY());
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

    /**
     * Compute distance between two Positions. Uses the distance formula.
     * @param pos1 First position
     * @param pos2 Second position.
     * @return A double with the distance between the two points.
     */
    public static double distance(Position pos1, Position pos2) {
        return Math.sqrt(Math.pow(pos1.getX() - pos2.getX(), 2) +
                         Math.pow(pos1.getY() - pos2.getY(), 2));
    }

    public static double angleBetween(Position pos1, Position pos2) {
        double heading = pos1.getHeading();
        return heading + Math.asin((pos2.getX() - pos1.getX()) / distance(pos1, pos2));
    }

    /**
     * Check if two positions are close enough to each other within a specified tolerance.
     * @param pos1 First position
     * @param pos2 Second position
     * @param tolerance Maximum distance between pos1 and pos2 to be considered close
     * @return Whether pos1 and pos2 are within tolerance distance of each other
     */
    public static boolean equalsWithinError(Position pos1, Position pos2, double tolerance) {
        return distance(pos1, pos2) < tolerance;
    }
}
