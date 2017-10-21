package main.java.com.cwrubotix.glennifer.automodule;

/**
 * Data type that represents a location inside the arena.
 * If angle is negative, the instance represents horizontal line in the arena.
 * 
 * @author Seohyun Jung
 *
 */

public class Position{

	private float x_pos;
	private float y_pos;
	/** 
	 * Represent the angle the robot is facing. EX) 0 when facing north, PI/2 when facing to the right and so forth (clock-wise).
	 * Range : [0, 2PI) unit in degrees.
	 * If angle is negative, the position represents a horizontal line. (For dividing up arena purpose)
	 * 
	 */
	private double angle; //declared double 'cause java.lang.Math hates float angles for trigonometry.
	private float tilt;
	private static final float WALL_CLEARANCE = 0.3F; //I set this to 30cm for now because I am scared of walls
	private static final float ARENA_WIDTH = 3.78F;  //+/- 1.39F From the middle (Tag is the origin)
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
		//checking whether input is within the arena
		if(x_pos < (ARENA_WIDTH() / 2) - WALL_CLEARANCE() && x_pos > (ARENA_WIDTH() / -2) + WALL_CLEARANCE()){
			this.x_pos = x_pos;
			return true;
		}
		return false;
	}
	
	public float getY(){
		return y_pos;
	}
	
	public boolean setY(float y_pos){
		//checking whether input is within the arena
		if(y_pos > WALL_CLEARANCE() && y_pos < 7.38F - WALL_CLEARANCE()){
			this.y_pos = y_pos;
			return true;
		}
		return false;
	}
	
	public double getAngle(){
		return angle;
	}
	
	public void setAngle(double angle){
		//checking whether angle is within the range
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
	
	/**
	 * Returns distance need to travel to position b
	 * @param b destination
	 * @return distance need to travel to position b
	 */
	public float getDistTo(Position b){
		return (float)Math.sqrt(Math.pow(getX() - b.getX(), 2) + Math.pow(getY() - b.getY(), 2));
	}
	
	/**
	 * Returns angle the robot need to turn to face position b
	 * @param b the destination
	 * @return angle the robot need to turn to face position b
	 */
	public double getAngleTurnTo(Position b){
		if(getAngle() >= 0 && b.getAngle() >= 0){
			return Math.PI - Math.atan((b.getX() - getX())/(b.getY() - getY())) - getAngle();
		}
		//Heading to vertically down (a.k.a. position b is straight horizontal line)
		return Math.PI - getAngle();
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
