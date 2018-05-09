package com.cwrubotix.glennifer.robot_state;

import java.time.Duration;
import java.time.Instant;
import java.util.EnumMap;
import java.util.Optional;

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
    public void 
    
    /* State getter methods */
   
    public LocalizationPosition getLocPosition(){

    }

    public List<ObstaclePosition> getObstacles(){
        
    }
}
