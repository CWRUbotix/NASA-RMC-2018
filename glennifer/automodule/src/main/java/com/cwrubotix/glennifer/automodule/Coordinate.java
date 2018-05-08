package com.cwrubotix.glennifer.automodule;

import java.util.Comparator;

/**
 * Abstract Data structure representing a coordinate within the arena
 * @author Seohyun Jung
 *
 */
public abstract class Coordinate{
    private double x_pos;
    private double y_pos;
    private static final double WALL_CLEARANCE = 0.80F; //Clearance distance from wall
    private static final double ARENA_WIDTH = 3.78F;  
    private static final double ARENA_HEIGHT = 7.38F; 
    
    public Coordinate(double x_pos, double y_pos){
	this.x_pos = x_pos;
	this.y_pos = y_pos;
    }
    
    public double getX() {
        return x_pos;
    }

    public boolean setX(double x_pos){
	if(x_pos < ARENA_WIDTH / 2 && x_pos > ARENA_WIDTH / -2){
	    this.x_pos = x_pos;
	    return true;
	}
	return false;
    }

    public double getY() {
        return y_pos;
    }

    public boolean setY(double y_pos){
	if(y_pos < ARENA_HEIGHT && y_pos >= 0){
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
    public double getDistTo(Coordinate b) {
        return Math.sqrt(Math.pow(getX() - b.getX(), 2) + Math.pow(getY() - b.getY(), 2));
    }
    
    public static Comparator<Coordinate> getComparatorByDistTo(final Coordinate pos) {
        return new Comparator<Coordinate>() {
            public int compare(Coordinate a, Coordinate b) {
                if (a.getDistTo(pos) < b.getDistTo(pos))
                    return -1;
                else if (a.getDistTo(pos) > b.getDistTo(pos))
                    return 1;
                else
                    return 0;
            }
        };
    }
    
    public static final double WALL_CLEARANCE() {
        return Coordinate.WALL_CLEARANCE;
    }

    public static final double ARENA_WIDTH() {
        return Coordinate.ARENA_WIDTH;
    }

    public static final double ARENA_HEIGHT() {
        return Coordinate.ARENA_HEIGHT;
    }
}
