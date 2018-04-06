package main.java.com.cwrubotix.glennifer.automodule;

public class Obstacle extends Coordinate {

    private float radius;
    private final float KINECT_ERROR_BOUND = 0.05F;

    public Obstacle(float x_pos, float y_pos, float radius) {
        super(x_pos, y_pos);
        this.radius = radius;
    }

    public float getRadius() {
        return radius;
    }

    public Obstacle(Position obs) {
        this(obs.getX(), obs.getY(), 0.15F);
    }

    public void setRadius(float radius) {
        this.radius = radius;
    }
    
    @Override
    public boolean equals(Object obj){
	if(obj instanceof Obstacle){
	    Obstacle other = (Obstacle)obj;
	    float x_diff = getX() - other.getX();
	    float y_diff = getY() - other.getY();
	    return x_diff <= KINECT_ERROR_BOUND && y_diff <= KINECT_ERROR_BOUND;
	}
	return false;
    }
}