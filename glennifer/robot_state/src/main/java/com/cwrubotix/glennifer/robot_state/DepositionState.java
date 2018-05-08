package com.cwrubotix.glennifer.robot_state;

import java.time.Instant;
import java.util.EnumMap;

/**
 * A DepositionState object encapsulates the current state of the robot's
 * deposition subsystem. It has update methods to give it sensor data, and
 * getter methods to query the state. Its update methods can raise fault
 * exceptions for all kinds of reasons. These faults can be responded to using
 * the adjustment method.
 * 
 * This class does not deal with messages or wire formats. It works purely at
 * the logical level.
 */
public class DepositionState {


    /**
     * The LoadCell enum is used to specify one of the deposition subsystem's 2
     * load cells.
     */
	//I guessed left and right, will update if wrong
    public enum LoadCell {
        LEFT,
        RIGHT
    }

    public enum Side {
        LEFT,
        RIGHT
    }
// Unneccesary since there are only two load cells??
    
    /* Data members */
    private EnumMap <LoadCell, Float> loadCellValue;
    private EnumMap <Side, Boolean> dumpSideRetracted;
    private EnumMap <Side, Boolean> dumpSideExtended;

    // TODO: Store the time most recently updated, either for the whole system
    // or for each sensor. If you want to handle out of order updates, you'll
    // need to do it for each sensor I think.
    
    /* Constructor */
    
    public DepositionState() {
        /* Implementation note: In this constructor, all data members are
         * initialized to 0 because this class does not currently consider the
         * case where it has never received input from a particular sensor. In
         * order to handle that case, initialization would need to be done
         * differently.
         */
        
        // TODO: handle no input from sensor

        loadCellValue = new EnumMap<>(LoadCell.class);
        loadCellValue.put(LoadCell.LEFT, (float)0);
        loadCellValue.put(LoadCell.RIGHT, (float)0);

        dumpSideRetracted = new EnumMap<>(Side.class);
        dumpSideRetracted.put(Side.LEFT, false);
        dumpSideRetracted.put(Side.RIGHT, false);

        dumpSideExtended = new EnumMap<>(Side.class);
        dumpSideExtended.put(Side.LEFT, false);
        dumpSideExtended.put(Side.RIGHT, false);

    }
    /* Update methods */
    
    public void updateDumpLoad (LoadCell cell, float load, Instant time) throws RobotFaultException {
        // TODO: use timestamp to validate data
        // TODO: detect impossibly sudden changes
        loadCellValue.put(cell, load);
    }
    
    public void updateDumpLimitExtended (Side side, boolean pressed, Instant time) throws RobotFaultException {
        dumpSideExtended.put(side, pressed);
    }
    
    public void updateDumpLimitRetracted (Side side, boolean pressed, Instant time) throws RobotFaultException {
        dumpSideRetracted.put(side, pressed);
    }
    
    /* State getter methods */
    
    public boolean isStored() {
        // TODO: use position too
        return getDumpRetracted();
    }
    
    public float getDumpLoad(LoadCell cell) {
        return loadCellValue.get(cell);
    }

    public float getDumpLoad() {
        //TODO: use linear functions to get more accurate reading
        return loadCellValue.get(LoadCell.LEFT) + loadCellValue.get(LoadCell.RIGHT);
    }

    public boolean getDumpExtended() { return dumpSideExtended.get(Side.LEFT) || dumpSideExtended.get(Side.RIGHT); }

    public boolean getDumpRetracted() { return dumpSideRetracted.get(Side.LEFT) || dumpSideRetracted.get(Side.RIGHT); }

    public boolean getDumpExtended(Side side) { return dumpSideExtended.get(side); }

    public boolean getDumpRetracted(Side side) { return dumpSideRetracted.get(side); }


}	