package com.cwrubotix.glennifer.robot_state;

import java.util.Optional;
import java.util.List;
import java.util.ArrayList;

import com.cwrubotix.glennifer.Messages.LocalizationPosition;
import com.cwrubotix.glennifer.Messages.ObstaclePosition;

/**
 * A LocomotionState object encapsulates the current state of the robot's
 * locomotion subsystem. It has update methods to give it sensor data, and
 * getter methods to query the state. Its update methods can raise fault
 * exceptions for all kinds of reasons. These faults can be responded to using
 * the adjustment method.
 * 
 * This class does not deal with messages or wire formats. It works purely at
 * the logical level.
 *
 *
 */
public class LocalizationObstacleState {

    

    /* Data members */
    LocalizationPosition robotPosition = LocalizationPosition.newBuilder()
                                                            .setXPosition(0)
                                                            .setYPosition(0)
                                                            .setBearingAngle(0)
                                                            .build();
    ArrayList<ObstaclePosition> obstacles = new ArrayList<ObstaclePosition>();
    
    /* Constructor */

    public LocalizationObstacleState() {
        /* Implementation note: In this constructor, all data members are
         * initialized to 0 because this class does not currently consider the
         * case where it has never received input from a particular sensor. In
         * order to handle that case, initialization would need to be done
         * differently.
         */

        // TODO: handle no input from sensor
    	//Initialize with empty optionals, if that's a thing
    }
    
    /* Update methods */
    public void updateLocalizationPosition(LocalizationPosition robotPosition) throws RobotFaultException {
            this.robotPosition = robotPosition;
        }
        // TODO: use timestamp to validate data
        // TODO: detect impossibly sudden changes

    public void addObstacle(ObstaclePosition obstaclePosition) throws RobotFaultException {
        float obstacleDistance = (float) Math.sqrt(Math.pow(obstaclePosition.getXPosition(), 2) + Math.pow(obstaclePosition.getYPosition(), 2));
        float obstacleAngle = (float) Math.atan(obstaclePosition.getYPosition() / obstaclePosition.getXPosition());
        ObstaclePosition newObstacle = ObstaclePosition.newBuilder()
                                                        .setXPosition((float)(robotPosition.getXPosition() + obstacleDistance * Math.cos(robotPosition.getBearingAngle() + obstacleAngle)))
                                                        .setYPosition((float)(robotPosition.getYPosition() + obstacleDistance * Math.sin(robotPosition.getBearingAngle() + obstacleAngle)))
                                                        .setZPosition(obstaclePosition.getZPosition())
                                                        .setDiameter(obstaclePosition.getDiameter())
                                                        .build();



        for(ObstaclePosition obstacle : obstacles){
            if((Math.abs(newObstacle.getXPosition() - obstacle.getXPosition()) < obstacle.getDiameter()) &&
                (Math.abs(newObstacle.getYPosition() - obstacle.getYPosition()) < obstacle.getDiameter())){
                return;
            }
        }
        obstacles.add(obstaclePosition);
        
    }
    
    /* State getter methods */
   
    public LocalizationPosition getLocPosition(){
        return robotPosition;
    }

    public List<ObstaclePosition> getObstacles(){
        return obstacles;
    }
}
