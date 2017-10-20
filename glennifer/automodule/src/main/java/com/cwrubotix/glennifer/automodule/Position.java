package main.java.com.cwrubotix.glennifer.automodule;

/**
 * Data type that represents a location inside the arena.
 * If angle is negative, the instance represents horizontal line in the arena.
 * 
 * @author Seohyun Jung
 *
 */

public class Position{
	/*
	 * TODO decide how/where we want to figure out the direction robot is facing
	 */
	private float x_pos;
	private float y_pos;
	/** 
	 * Represent the angle the robot is facing. EX) 0 when facing north, 90 when facing to the right and so forth (clock-wise).
	 * Range : [0, 360) unit in degrees.
	 * If angle is negative, the position represents a horizontal line. (For dividing up arena purpose)
	 * 
	 */
	private double angle; //declared double 'cause java.lang.Math hates float angles for trigonometry.
	private float tilt; //TODO should debate on whether we need this.
	private static final float WALL_CLEARANCE = 0.3F; //I set this to 30cm for now because I am scared of walls
	private static final float ARENA_WIDTH = 3.78F;
	private static final float ARENA_HEIGHT = 7.38F;
	
	
	public Position(float x_pos, float y_pos, double angle, float tilt){
		this.x_pos = x_pos;
		this.y_pos = y_pos;
		this.angle = angle;
		this.tilt = tilt;
	}
	
	public float getX(){
		return x_pos;
	}
	
	public boolean setX(float x_pos){
		if(x_pos < ARENA_WIDTH() - WALL_CLEARANCE() && x_pos > WALL_CLEARANCE()){
			this.x_pos = x_pos;
			return true;
		}
		return false;
	}
	
	public float getY(){
		return y_pos;
	}
	
	public boolean setY(float y_pos){
		if(y_pos >= 0.0F && y_pos < 7.38F - WALL_CLEARANCE()){
			this.y_pos = y_pos;
			return true;
		}
		return false;
	}
	
	public double getAngle(){
		return angle;
	}
	
	public void setAngle(double angle){
		if(angle >= Math.PI * 2){
			this.angle = angle - Math.PI * 2;
		}
		else{
			this.angle = angle;
		}
	}
	
	public float getTilt(){
		return tilt;
	}
	
	public void setTilt(float tilt){
		this.tilt = tilt;
	}
	
	public static float getDist(Position a, Position b){
		return (float)Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
	}
	
	/*TODO*/
	public double getAngleRespectTo(Position p){
		return 0.0;
	}
	
	public static final float WALL_CLEARANCE(){
		return Position.WALL_CLEARANCE;
	}
	
	public static final float ARENA_WIDTH(){
		return Position.ARENA_WIDTH;
	}
	
	public static final float ARENA_HEIGHT(){
		return Position.ARENA_HEIGHT;
	}
	
}
