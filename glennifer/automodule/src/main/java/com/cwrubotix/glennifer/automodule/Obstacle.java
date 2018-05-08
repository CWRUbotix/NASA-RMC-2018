package com.cwrubotix.glennifer.automodule;

public class Obstacle extends Coordinate {

    private double radius;
    private final double KINECT_ERROR_BOUND = 0.05F;

    public Obstacle(double x_pos, double y_pos, double radius) {
        super(x_pos, y_pos);
        this.radius = radius;
    }

    public double getRadius() {
        return radius;
    }

    public Obstacle(Position obs) {
        this(obs.getX(), obs.getY(), 0.15F);
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }
    
    @Override
    public boolean equals(Object obj){
	if(obj instanceof Obstacle){
	    Obstacle other = (Obstacle)obj;
	    double x_diff = getX() - other.getX();
	    double y_diff = getY() - other.getY();
	    double r_diff = getRadius() - other.getRadius();
	    return x_diff <= KINECT_ERROR_BOUND && y_diff <= KINECT_ERROR_BOUND && r_diff <= KINECT_ERROR_BOUND;
	}
	return false;
    }
    
    @Override
    public String toString(){
	return "(" + getX() + ", " + getY() + " r: " + getRadius() + ")";
    }
}
