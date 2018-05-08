package com.cwrubotix.glennifer.robot_state;

import java.time.Duration;
import java.time.Instant;
import java.util.EnumMap;
import java.util.Optional;

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
public class LocomotionState {

    /**
     * The Wheel enum is used to specify one of the locomotion subsystem's 4
     * wheels.
     */
    public enum Wheel {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    /* Data members */
    private EnumMap<Wheel, Optional<Float>> wheelRpm;
    private EnumMap<Wheel, Optional<Integer>> wheelCount;

    // TODO: Store the time most recently updated, either for the whole system
    // or for each sensor. If you want to handle out of order updates, you'll
    // need to do it for each sensor I think.

    //Instants for containing time since updates
    private Instant timeSinceWheelRPM;
    private Instant timeSystem;    
    
    /* Constructor */

    public LocomotionState() {
        /* Implementation note: In this constructor, all data members are
         * initialized to 0 because this class does not currently consider the
         * case where it has never received input from a particular sensor. In
         * order to handle that case, initialization would need to be done
         * differently.
         */

        // TODO: handle no input from sensor
    	//Initialize with empty optionals, if that's a thing
        wheelRpm = new EnumMap<>(Wheel.class);
        wheelRpm.put(Wheel.FRONT_LEFT, Optional.empty());
        wheelRpm.put(Wheel.FRONT_RIGHT, Optional.empty());
        wheelRpm.put(Wheel.BACK_LEFT, Optional.empty());
        wheelRpm.put(Wheel.BACK_RIGHT, Optional.empty());
        
        wheelCount = new EnumMap<>(Wheel.class); 
        wheelCount.put(Wheel.FRONT_LEFT, Optional.empty());
        wheelCount.put(Wheel.FRONT_RIGHT, Optional.empty());
        wheelCount.put(Wheel.BACK_LEFT, Optional.empty());
        wheelCount.put(Wheel.BACK_RIGHT, Optional.empty());
    }
    
    /* Update methods */
    //when this is called, rpm will never be null
    public void updateWheelRpm(Wheel wheel, float rpm, Instant time) throws RobotFaultException {
        // TODO: use timestamp to validate data
        // TODO: detect impossibly sudden changes

        wheelRpm.put(wheel, Optional.of(rpm));

        //Check if time is null
        Optional<Instant> opTime = Optional.ofNullable(time);
        if ((opTime.isPresent()) && timeSinceWheelRPM != null) {
            Duration duration = Duration.between(time, timeSinceWheelRPM);

            //some given consistency value?
            if (duration.toMillis() > 2000) {
                //throw something?
            }
        }
        //else update PodPos time
        timeSinceWheelRPM = Instant.now();
    }
    public void updateWheelCount(Wheel wheel, int counts, Instant time) throws RobotFaultException {
    	//Note: wheel count is the output from the encoder, may need processing to be a valid unit
    	
    	wheelCount.put(wheel, Optional.of(counts));
    }
    
    /* State getter methods */
   
    public float getSpeed() {
        // TODO: use physical constants, real or made up, to get speed

        //number of wheels reporting values
        int divNum = 0;
        //total RPM for forward speed
        Float rpmTot = (float) 0;
        
        Optional<Float> rpmWL = wheelRpm.get(Wheel.FRONT_LEFT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmTot += rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.FRONT_RIGHT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmTot += rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.BACK_LEFT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmTot += rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.BACK_RIGHT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmTot += rpmWL.get();
        }
        //speed is the averaged RPM for reporting wheels
        return rpmTot / divNum;
    }

    public int getWheelCount(Wheel wheel){
        if (wheelCount.get(wheel).isPresent()){
            return wheelCount.get(wheel).get();
        }
        return 0;
    }

    //
    public float getWheelRpm(Wheel wheel) {
        if (wheelRpm.get(wheel).isPresent()){
            return wheelRpm.get(wheel).get();
        }   
        return 0f;
    }
}
