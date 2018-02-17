package main.java.com.cwrubotix.glennifer.automodule;

import java.util.Comparator;

public abstract class Coordinate{
    private float x_pos;
    private float y_pos;
    private static final float WALL_CLEARANCE = 0.80F; 
    private static final float ARENA_WIDTH = 3.78F;  //+/- 1.39F From the middle (Tag is the origin)
    private static final float ARENA_HEIGHT = 7.38F;
    
    public Coordinate(float x_pos, float y_pos){
	this.x_pos = x_pos;
	this.y_pos = y_pos;
    }
    
    public float getX() {
        return x_pos;
    }

    public boolean setX(float x_pos){
	if(x_pos < ARENA_WIDTH / 2 && x_pos > ARENA_WIDTH / -2){
	    this.x_pos = x_pos;
	    return true;
	}
	return false;
    }

    public float getY() {
        return y_pos;
    }

    public boolean setY(float y_pos){
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
    public float getDistTo(Coordinate b) {
        return (float) Math.sqrt(Math.pow(getX() - b.getX(), 2) + Math.pow(getY() - b.getY(), 2));
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
    
    public static final float WALL_CLEARANCE() {
        return Coordinate.WALL_CLEARANCE;
    }

    public static final float ARENA_WIDTH() {
        return Coordinate.ARENA_WIDTH;
    }

    public static final float ARENA_HEIGHT() {
        return Coordinate.ARENA_HEIGHT;
    }
}