package main.java.com.cwrubotix.glennifer.automodule;

public class Obstacle extends Position{

	private float diameter;
	
	public Obstacle(float x_pos, float y_pos, float diameter) {
		super(x_pos, y_pos, 0.0F, 0.0F);
		this.diameter = diameter;
	}
	
	public Obstacle(Position obs) {
    	    this(obs.getX(), obs.getY(), 0.4F);
	}

	public float getDiameter(){
		return diameter;
	}
	
	public void setDiameter(float diameter){
		this.diameter = diameter;
	}
	
}