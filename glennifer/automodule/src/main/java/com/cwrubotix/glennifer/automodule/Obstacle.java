package com.cwrubotix.glennifer.automodule;

public class Obstacle extends Position {

	private float radius;
	
	public Obstacle(float x_pos, float y_pos, float radius) {
		super(x_pos, y_pos, 0.0F, 0.0F);
		this.radius = radius;
	}
	
	public float getRadius(){
		return radius;
	}
	
	public void setRadius(float radius){
		this.radius = radius;
	}

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Position) {
            Position pos = (Position) obj;
            return this.getDistTo(pos) <= radius || pos.equals(this);
        }
        return false;
    }
}