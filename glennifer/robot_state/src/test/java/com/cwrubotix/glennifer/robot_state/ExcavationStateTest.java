package com.cwrubotix.glennifer.robot_state;

import java.time.Instant;
import org.junit.Test;
import static org.junit.Assert.*;

public class ExcavationStateTest {

    public ExcavationStateTest() { }

    /**
     * Test of updateConveyorRpm method, of class ExcavationState.
     */
    @Test
    public void testConveyorRpm() throws Exception {
        float rpmConveyor = 4.1F;
        Instant time = Instant.now();
        ExcavationState instance = new ExcavationState();
        instance.updateConveyorRpm(rpmConveyor, time);
        float resultConveyor = instance.getConveyorRpm();
        assertEquals(rpmConveyor, resultConveyor, 0);
    }

    /**
     * Test of updateArmPos method, of class ExcavationState.
     */
    @Test
    public void testArmPos() throws Exception {
        float armPos = 2.7F;
        Instant time = Instant.now();
        ExcavationState instance = new ExcavationState();
        instance.updateArmPos(armPos, time);
        float resultArmPos = instance.getArmPos();
        assertEquals(armPos, resultArmPos, 0);
    }

    /**
     * Test of updateTranslationDisplacement method, of class ExcavationState.
     */
    @Test
    public void testConveyorTranslationDisplacement() throws Exception {
        float disp = 3.5F;
        Instant time = Instant.now();
        ExcavationState instance = new ExcavationState();
        instance.updateTranslationDisplacement(disp, time);
        float resultDisplacement = instance.getTranslationDisplacement();
        assertEquals(disp, resultDisplacement, 0);
    }

    @Test
    public void testLimitSwitches() throws Exception {
        Instant time = Instant.now();
        ExcavationState instance = new ExcavationState();
        instance.updateArmLimitExtended(ExcavationState.Side.LEFT, true, time);
        assertTrue(instance.getArmExtended(ExcavationState.Side.LEFT));
        assertTrue(instance.getArmExtended());
        instance.updateArmLimitExtended(ExcavationState.Side.LEFT, false, time);
        assertFalse(instance.getArmExtended(ExcavationState.Side.LEFT));
        assertFalse(instance.getArmExtended());
    }
}
