package com.cwrubotix.glennifer.robot_state;

/**
 * This exception class is to be thrown by methods dealing with the robot logic
 * when they need to raise a fault code. Code 0 is reserved.
 */
public class RobotFaultException extends Exception {
    
    private int faultCode;
    
    public RobotFaultException(int faultCode) {
        super("Robot Fault Code " + Integer.toString(faultCode));
        this.faultCode = faultCode;
    }
    
    public RobotFaultException() {
        this(0);
    }
    
    public int getFaultCode() {
        return this.faultCode;
    }
}
