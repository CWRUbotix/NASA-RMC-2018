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

    private static final float WHEEL_POD_POS_MAX_STRAIGHT = 10f;
    private static final float WHEEL_POD_POS_MIN_TURN = 40f;
    private static final float WHEEL_POD_POS_MAX_TURN = 50f;
    private static final float WHEEL_POD_POS_MIN_STRAFE = 80f;

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

    /**
     * The Configuration enum is used to represent the locomotion subsystem's
     * overall wheel pod configuration.
     */
    public enum Configuration {
        INTERMEDIATE,
        STRAIGHT,
        TURN,
        STRAFE
    }

    /* Data members */
    private EnumMap<Wheel, Optional<Float>> wheelRpm;
    private EnumMap<Wheel, Float> wheelPodPos;
    private EnumMap<Wheel, Boolean> wheelPodLimitRetracted;
    private EnumMap<Wheel, Boolean> wheelPodLimitExtended;

    // TODO: Store the time most recently updated, either for the whole system
    // or for each sensor. If you want to handle out of order updates, you'll
    // need to do it for each sensor I think.

    //Floats for containing time since updates
    private Instant timeSinceWheelRPM;
    private Instant timeSincePodPos;
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

        wheelPodPos = new EnumMap<>(Wheel.class);
        wheelPodPos.put(Wheel.FRONT_LEFT, (float) 0);
        wheelPodPos.put(Wheel.FRONT_RIGHT, (float) 0);
        wheelPodPos.put(Wheel.BACK_LEFT, (float) 0);
        wheelPodPos.put(Wheel.BACK_RIGHT, (float) 0);

        wheelPodLimitRetracted = new EnumMap<>(Wheel.class);
        wheelPodLimitRetracted.put(Wheel.FRONT_LEFT, false);
        wheelPodLimitRetracted.put(Wheel.FRONT_RIGHT, false);
        wheelPodLimitRetracted.put(Wheel.BACK_LEFT, false);
        wheelPodLimitRetracted.put(Wheel.BACK_RIGHT, false);

        wheelPodLimitExtended = new EnumMap<>(Wheel.class);
        wheelPodLimitExtended.put(Wheel.FRONT_LEFT, false);
        wheelPodLimitExtended.put(Wheel.FRONT_RIGHT, false);
        wheelPodLimitExtended.put(Wheel.BACK_LEFT, false);
        wheelPodLimitExtended.put(Wheel.BACK_RIGHT, false);
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
    
    public void updateWheelPodPos (Wheel wheel, float pos, Instant time) throws RobotFaultException {
        // TODO: use timestamp to validate data
        // TODO: detect impossibly sudden changes
    	
    	if(timeSincePodPos != null && time != null){
    	Duration duration = Duration.between(time, timeSincePodPos);
    	//Placeholder for now, but eventually see if values are ridiculous
    		if(duration.toMillis() > 2000){
    		//throw something?
    		}
    	}
    	//else update PodPos time
    	timeSincePodPos = Instant.now();
        wheelPodPos.put(wheel, pos);
    }
    
    public void updateWheelPodLimitExtended (Wheel wheel, boolean pressed, Instant time) throws RobotFaultException {
    	 // TODO: use limit 
        // From Paul:
        //For Locomotion, there is only one limit switch. 
        //When it is fully retracted, we know we are in STRAIGHT
        //When it is fully extended, we know we are in STRAFE
        //Turning configuration is somewhere in between, we can't tell with the limit switches.
        //For all situations, we should apparently use the potentiometer to double check
        wheelPodLimitExtended.put(wheel, pressed);
    }
    
    public void updateWheelPodLimitRetracted (Wheel wheel, boolean pressed, Instant time) throws RobotFaultException {
        // TODO: use limit switches
        wheelPodLimitRetracted.put(wheel, pressed);
    }
    
    /* State getter methods */
    
    public Configuration getConfiguration() {
        // TODO: use real physical constants to get configuration
        Configuration lastConfig = null;
        for (Wheel wheel : Wheel.values()) {

            // Determined the current wheel's individual config
            float podPos = wheelPodPos.get(wheel);
            boolean retracted = wheelPodLimitRetracted.get(wheel);
            boolean extended = wheelPodLimitExtended.get(wheel);
            Configuration currentConfig = Configuration.INTERMEDIATE;
            if (retracted) {
                currentConfig = Configuration.STRAIGHT;
            } else if (extended) {
                currentConfig = Configuration.STRAFE;
            } else if (podPos <= WHEEL_POD_POS_MAX_STRAIGHT) {
                currentConfig = Configuration.STRAIGHT;
            } else if (podPos >= WHEEL_POD_POS_MIN_STRAFE) {
                currentConfig = Configuration.STRAFE;
            } else if (podPos <= WHEEL_POD_POS_MAX_TURN && podPos >= WHEEL_POD_POS_MIN_TURN) {
                currentConfig = Configuration.TURN;
            }

            // Compare to last config
            if (currentConfig == Configuration.INTERMEDIATE) {
                return currentConfig;
            }
            if (lastConfig != null) {
                if (currentConfig != lastConfig) {
                    return Configuration.INTERMEDIATE;
                }
            }
            lastConfig = currentConfig;
        }
        return lastConfig;
    }
    
    public float getStraightSpeed() {
        // TODO: use physical constants, real or made up, to get speed

        //number of wheels reporting values
        int divNum = 0;
        //total RPM for forward speed
        Float rpmTot = (float) 0;
        /// For TURN: Left wheels are positive, right wheels are negative
        /// For STRAFE: FL and BR are positive, FR and BL are negative
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
    
    public float getTurnSpeed() {
        // TODO: use physical constants, real or made up, to get speed

        //number of wheels reporting values
        int divNum = 0;
        Float rpmTurn = (float) 0;
        /// For TURN: Left wheels are positive, right wheels are negative
        /// For STRAFE: FL and BR are positive, FR and BL are negative
        Optional<Float> rpmWL = wheelRpm.get(Wheel.FRONT_LEFT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmTurn += rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.FRONT_RIGHT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmTurn -= rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.BACK_LEFT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmTurn += rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.BACK_RIGHT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmTurn -= rpmWL.get();
        }
        //speed is the averaged RPM for reporting wheels
        return rpmTurn / divNum;
    }
    
    public float getStrafeSpeed() {
        // TODO: use physical constants, real or made up, to get speed

        //number of wheels reporting values
        int divNum = 0;
        Float rpmStrafe = (float) 0;
        /// For STRAFE: FL and BR are positive, FR and BL are negative
        Optional<Float> rpmWL = wheelRpm.get(Wheel.FRONT_LEFT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmStrafe += rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.FRONT_RIGHT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmStrafe -= rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.BACK_LEFT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmStrafe -= rpmWL.get();
        }
        rpmWL = wheelRpm.get(Wheel.BACK_RIGHT);
        if (rpmWL.isPresent()) {
            divNum++;
            rpmStrafe += rpmWL.get();
        }
        //speed is the averaged RPM for reporting wheels
        return rpmStrafe / divNum;
    }
    
    public float getWheelRpm(Wheel wheel) {
        return wheelRpm.get(wheel).get();
    }
    
    public float getWheelPodPos(Wheel wheel) {
        return wheelPodPos.get(wheel);
    }

    public boolean getWheelPodLimitRetracted(Wheel wheel) {
        return wheelPodLimitRetracted.get(wheel);
    }

    public boolean getWheelPodLimitExtended(Wheel wheel) {
        return wheelPodLimitExtended.get(wheel);
    }
}
