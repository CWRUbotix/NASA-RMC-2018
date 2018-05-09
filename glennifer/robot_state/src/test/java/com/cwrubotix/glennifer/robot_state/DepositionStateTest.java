package com.cwrubotix.glennifer.robot_state;

import java.time.Instant;
import org.junit.Test;
import static org.junit.Assert.*;

public class DepositionStateTest {

    public DepositionStateTest() { }

    /**
     * Test of updateDumpLoad method, of class DepositionState.
     */
    @Test
    public void testDumpLoad() throws Exception {
        float load = 4.1F;
        Instant time = Instant.now();
        DepositionState instance = new DepositionState();
        DepositionState.LoadCell cell = DepositionState.LoadCell.LEFT;
        instance.updateDumpLoad(cell, load, time);
        float resultLoad = instance.getDumpLoad(DepositionState.LoadCell.LEFT);
        assertEquals(load, resultLoad, 0);
    }

    @Test
    public void testLimitSwitches() throws Exception {
        Instant time = Instant.now();
        DepositionState instance = new DepositionState();
        instance.updateDumpLimitExtended(DepositionState.Side.LEFT, true, time);
        assertTrue(instance.getDumpExtended(DepositionState.Side.LEFT));
        assertTrue(instance.getDumpExtended());
        instance.updateDumpLimitExtended(DepositionState.Side.LEFT, false, time);
        assertFalse(instance.getDumpExtended(DepositionState.Side.LEFT));
        assertFalse(instance.getDumpExtended());
    }

    /**
     * Test of updateArmLimitExtended, UpdateArmLimitRetracted methods, of class DepositionState.
     */
    @Test
    public void testLimitSwitchConfigurations() throws Exception {
        Instant time = Instant.now();
        DepositionState instance = new DepositionState();
        instance.updateDumpLimitRetracted(DepositionState.Side.LEFT, true, time); // Retracted
        assertTrue(instance.isStored());
        instance.updateDumpLimitRetracted(DepositionState.Side.LEFT, false, time); // Not retracted
        assertFalse(instance.isStored());
    }
}

