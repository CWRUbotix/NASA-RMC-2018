package main.java.com.cwrubotix.glennifer.automodule;

public class RobotPosition extends Position{
    /**
     * <p>
     * Represent the angle the robot is facing. EX) 0 when facing north, PI/2 when facing to the right and so forth (clock-wise).
     * </p>
     * <p>
     * Range : [0, 2PI) unit in radians.
     * </p>
     * If angle is negative, the position represents a horizontal line. (For dividing up arena purpose)
     */
    private double angle; //declared double 'cause java.lang.Math hates float angles for trigonometry.
    private float tilt;
    
    
    public RobotPosition(float x_pos, float y_pos, double angle, float tilt){
	super(x_pos, y_pos);
	this.angle = angle;
	this.tilt = tilt;
    }
    
    public RobotPosition(float x_pos, float y_pos){
	super(x_pos, y_pos);
    }
    
    public double getAngle(){
	return angle;
    }
    
    public float getTilt(){
	return tilt;
    }
    
    public void setAngle(double angle){
	this.angle = angle;
    }
    
    public void setTilt(float tilt){
	this.tilt = tilt;
    }
    
    /**
     * Returns angle the robot need to be in order to face position b
     *
     * @param b the destination
     * @return angle the robot need to be in order to face position b
     */
    public double getAngleTurnTo(RobotPosition b) {
        if (getAngle() >= 0 && b.getAngle() >= 0) {
            float x_diff = b.getX() - getX();
            float y_diff = b.getY() - getY();
            if (x_diff < 0) {
                return Math.PI + Math.PI / 2 - Math.atan((double) (y_diff / x_diff));
            } else {
                return Math.PI / 2 - Math.atan((double) (y_diff / x_diff));
            }
        }
        //Heading to vertically down (a.k.a. position b is straight horizontal line)
        return Math.PI;
    }
    
    public double getAngleTurnTo(Position p){
	if(getAngle() < 0)
	    return Math.PI;
	float x_diff = p.getX() - getX();
        float y_diff = p.getY() - getY();
        if (x_diff < 0) {
            return Math.PI + Math.PI / 2 - Math.atan((double) (y_diff / x_diff));
        } else {
            return Math.PI / 2 - Math.atan((double) (y_diff / x_diff));
        }
    }
}