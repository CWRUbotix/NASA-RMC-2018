package com.cwrubotix.glennifer.robot_state;

import java.time.Instant;
import org.junit.Test;
import static org.junit.Assert.*;

public class LocomotionStateTest {
    
    public LocomotionStateTest() { }

    /**
     * Test of updateWheelRpm method, of class LocomotionState.
     */
    @Test
    public void testWheelRpm() throws Exception {
        float rpmFrontLeft = 4.2F;
        Instant time = Instant.now();
        LocomotionState instance = new LocomotionState();
        instance.updateWheelRpm(LocomotionState.Wheel.FRONT_LEFT, rpmFrontLeft, time);
        float resultFrontLeft = instance.getWheelRpm(LocomotionState.Wheel.FRONT_LEFT);
        assertEquals(rpmFrontLeft, resultFrontLeft, 0);
        float rpmFrontRight = 5.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.FRONT_RIGHT, rpmFrontRight, time);
        resultFrontLeft = instance.getWheelRpm(LocomotionState.Wheel.FRONT_LEFT);
        float resultFrontRight = instance.getWheelRpm(LocomotionState.Wheel.FRONT_RIGHT);
        assertEquals(rpmFrontLeft, resultFrontLeft, 0);
        assertEquals(rpmFrontRight, resultFrontRight, 0);
        float rpmBackLeft = -6.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.BACK_LEFT, rpmBackLeft, time);
        resultFrontLeft = instance.getWheelRpm(LocomotionState.Wheel.FRONT_LEFT);
        resultFrontRight = instance.getWheelRpm(LocomotionState.Wheel.FRONT_RIGHT);
        float resultBackLeft = instance.getWheelRpm(LocomotionState.Wheel.BACK_LEFT);
        assertEquals(rpmFrontLeft, resultFrontLeft, 0);
        assertEquals(rpmFrontRight, resultFrontRight, 0);
        assertEquals(rpmBackLeft, resultBackLeft, 0);
        float rpmBackRight = -1.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.BACK_RIGHT, rpmBackRight, time);
        resultFrontLeft = instance.getWheelRpm(LocomotionState.Wheel.FRONT_LEFT);
        resultFrontRight = instance.getWheelRpm(LocomotionState.Wheel.FRONT_RIGHT);
        resultBackLeft = instance.getWheelRpm(LocomotionState.Wheel.BACK_LEFT);
        float resultBackRight = instance.getWheelRpm(LocomotionState.Wheel.BACK_RIGHT);
        assertEquals(rpmFrontLeft, resultFrontLeft, 0);
        assertEquals(rpmFrontRight, resultFrontRight, 0);
        assertEquals(rpmBackLeft, resultBackLeft, 0);
        assertEquals(rpmBackRight, resultBackRight, 0);
    }

    @Test
    public void testNullSpeed() throws Exception {
    	Instant time = Instant.now();
        LocomotionState instance = new LocomotionState();    
        float resultAverageStraightSpeed = instance.getStraightSpeed();
        float resultAverageTurnSpeed = instance.getTurnSpeed();
        float resultAverageStrafeSpeed = instance.getStrafeSpeed();
        float averageStraightRpm = 0.0F;
        float averageTurnRpm = 0.0F;
        float averageStrafeRpm = 0.0F;
        //Expected behavior is now to expect a NaN if the wheels are not reporting
        assertEquals(Float.NaN, resultAverageStraightSpeed, 0);
        assertEquals(Float.NaN, resultAverageTurnSpeed, 0);
        assertEquals(Float.NaN, resultAverageStrafeSpeed, 0);
        
      //Update 3 Wheels, Don't for wheel front right - Not sure if situation should be handled differently though.
        
        float rpmFrontLeft = 4.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.FRONT_LEFT, rpmFrontLeft, time);
        //float rpmFrontRight = 5.2F;
        //instance.updateWheelRpm(LocomotionState.Wheel.FRONT_RIGHT, rpmFrontRight, time);
        float rpmBackLeft = -6.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.BACK_LEFT, rpmBackLeft, time);
        float rpmBackRight = -1.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.BACK_RIGHT, rpmBackRight, time);
        averageStraightRpm = (rpmFrontLeft + rpmBackLeft + rpmBackRight) / 3;
        averageTurnRpm = (rpmFrontLeft + rpmBackLeft - rpmBackRight) / 3;
        averageStrafeRpm = (rpmFrontLeft - rpmBackLeft + rpmBackRight) / 3;
        resultAverageStraightSpeed = instance.getStraightSpeed();
        resultAverageTurnSpeed = instance.getTurnSpeed();
        resultAverageStrafeSpeed = instance.getStrafeSpeed();
        
        assertEquals(averageStraightRpm, resultAverageStraightSpeed, 0);
        assertEquals(averageTurnRpm, resultAverageTurnSpeed, 0);
        assertEquals(averageStrafeRpm, resultAverageStrafeSpeed, 0);
    }
    
    @Test
    public void testSpeed() throws Exception {
        float rpmFrontLeft = 4.2F;
        Instant time = Instant.now();
        LocomotionState instance = new LocomotionState();
        instance.updateWheelRpm(LocomotionState.Wheel.FRONT_LEFT, rpmFrontLeft, time);
        float rpmFrontRight = 5.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.FRONT_RIGHT, rpmFrontRight, time);
        float rpmBackLeft = -6.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.BACK_LEFT, rpmBackLeft, time);
        float rpmBackRight = -1.2F;
        instance.updateWheelRpm(LocomotionState.Wheel.BACK_RIGHT, rpmBackRight, time);
        float averageStraightRpm = (rpmFrontLeft + rpmFrontRight + rpmBackLeft + rpmBackRight) / 4;
        float averageTurnRpm = (rpmFrontLeft - rpmFrontRight + rpmBackLeft - rpmBackRight) / 4;
        float averageStrafeRpm = (rpmFrontLeft - rpmFrontRight - rpmBackLeft + rpmBackRight) / 4;
        float resultAverageStraightSpeed = instance.getStraightSpeed();
        float resultAverageTurnSpeed = instance.getTurnSpeed();
        float resultAverageStrafeSpeed = instance.getStrafeSpeed();
        double delta = 1e-6;
        assertEquals(averageStraightRpm, resultAverageStraightSpeed, delta);
        assertEquals(averageTurnRpm, resultAverageTurnSpeed, 0);
        assertEquals(averageStrafeRpm, resultAverageStrafeSpeed, 0);
    }

    /**
     * Test of updateWheelPodPos method, of class LocomotionState.
     */
    @Test
    public void testWheelPodPos() throws Exception {
        float podPosFrontLeft = 4.2F;
        Instant time = Instant.now();
        LocomotionState instance = new LocomotionState();
        instance.updateWheelPodPos(LocomotionState.Wheel.FRONT_LEFT, podPosFrontLeft, time);
        float resultFrontLeft = instance.getWheelPodPos(LocomotionState.Wheel.FRONT_LEFT);
        assertEquals(podPosFrontLeft, resultFrontLeft, 0);
        float podPosFrontRight = 5.2F;
        instance.updateWheelPodPos(LocomotionState.Wheel.FRONT_RIGHT, podPosFrontRight, time);
        resultFrontLeft = instance.getWheelPodPos(LocomotionState.Wheel.FRONT_LEFT);
        float resultFrontRight = instance.getWheelPodPos(LocomotionState.Wheel.FRONT_RIGHT);
        assertEquals(podPosFrontLeft, resultFrontLeft, 0);
        assertEquals(podPosFrontRight, resultFrontRight, 0);
        float podPosBackLeft = -6.2F;
        instance.updateWheelPodPos(LocomotionState.Wheel.BACK_LEFT, podPosBackLeft, time);
        resultFrontLeft = instance.getWheelPodPos(LocomotionState.Wheel.FRONT_LEFT);
        resultFrontRight = instance.getWheelPodPos(LocomotionState.Wheel.FRONT_RIGHT);
        float resultBackLeft = instance.getWheelPodPos(LocomotionState.Wheel.BACK_LEFT);
        assertEquals(podPosFrontLeft, resultFrontLeft, 0);
        assertEquals(podPosFrontRight, resultFrontRight, 0);
        assertEquals(podPosBackLeft, resultBackLeft, 0);
        float podPosBackRight = -1.2F;
        instance.updateWheelPodPos(LocomotionState.Wheel.BACK_RIGHT, podPosBackRight, time);
        resultFrontLeft = instance.getWheelPodPos(LocomotionState.Wheel.FRONT_LEFT);
        resultFrontRight = instance.getWheelPodPos(LocomotionState.Wheel.FRONT_RIGHT);
        resultBackLeft = instance.getWheelPodPos(LocomotionState.Wheel.BACK_LEFT);
        float resultBackRight = instance.getWheelPodPos(LocomotionState.Wheel.BACK_RIGHT);
        assertEquals(podPosFrontLeft, resultFrontLeft, 0);
        assertEquals(podPosFrontRight, resultFrontRight, 0);
        assertEquals(podPosBackLeft, resultBackLeft, 0);
        assertEquals(podPosBackRight, resultBackRight, 0);
    }

    @Test
    public void testLimitSwitches() throws Exception {
        Instant time = Instant.now();
        LocomotionState instance = new LocomotionState();
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.BACK_LEFT, true, time);
        assertTrue(instance.getWheelPodLimitExtended(LocomotionState.Wheel.BACK_LEFT));
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.BACK_RIGHT, true, time);
        assertTrue(instance.getWheelPodLimitExtended(LocomotionState.Wheel.BACK_RIGHT));
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.FRONT_LEFT, true, time);
        assertTrue(instance.getWheelPodLimitExtended(LocomotionState.Wheel.FRONT_LEFT));
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.FRONT_RIGHT, true, time);
        assertTrue(instance.getWheelPodLimitExtended(LocomotionState.Wheel.FRONT_RIGHT));

        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.BACK_LEFT, false, time);
        assertFalse(instance.getWheelPodLimitExtended(LocomotionState.Wheel.BACK_LEFT));
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.BACK_RIGHT, false, time);
        assertFalse(instance.getWheelPodLimitExtended(LocomotionState.Wheel.BACK_RIGHT));
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.FRONT_LEFT, false, time);
        assertFalse(instance.getWheelPodLimitExtended(LocomotionState.Wheel.FRONT_LEFT));
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.FRONT_RIGHT, false, time);
        assertFalse(instance.getWheelPodLimitExtended(LocomotionState.Wheel.FRONT_RIGHT));

        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.BACK_LEFT, true, time);
        assertTrue(instance.getWheelPodLimitRetracted(LocomotionState.Wheel.BACK_LEFT));
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.BACK_RIGHT, true, time);
        assertTrue(instance.getWheelPodLimitRetracted(LocomotionState.Wheel.BACK_RIGHT));
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_LEFT, true, time);
        assertTrue(instance.getWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_LEFT));
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_RIGHT, true, time);
        assertTrue(instance.getWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_RIGHT));

        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.BACK_LEFT, false, time);
        assertFalse(instance.getWheelPodLimitRetracted(LocomotionState.Wheel.BACK_LEFT));
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.BACK_RIGHT, false, time);
        assertFalse(instance.getWheelPodLimitRetracted(LocomotionState.Wheel.BACK_RIGHT));
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_LEFT, false, time);
        assertFalse(instance.getWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_LEFT));
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_RIGHT, false, time);
        assertFalse(instance.getWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_RIGHT));
    }

    /**
     * Test of updateWheelPodLimitExtended and UpdateWheelPodLimitRetracted methods, of class LocomotionState.
     */
    @Test
    public void testLimitSwitchConfigurations() throws Exception {
        Instant time = Instant.now();
        LocomotionState instance = new LocomotionState();
        // Start in STRAIGHT configuration
        assertEquals(LocomotionState.Configuration.STRAIGHT, instance.getConfiguration());
        // Enter STRAFE configuration
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.BACK_LEFT, true, time);
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.BACK_RIGHT, true, time);
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.FRONT_LEFT, true, time);
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.FRONT_RIGHT, true, time);
        assertEquals(LocomotionState.Configuration.STRAFE, instance.getConfiguration());
        // Enter STRAIGHT configuration
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.BACK_LEFT, false, time);
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.BACK_RIGHT, false, time);
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.FRONT_LEFT, false, time);
        instance.updateWheelPodLimitExtended(LocomotionState.Wheel.FRONT_RIGHT, false, time);
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.BACK_LEFT, true, time);
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.BACK_RIGHT, true, time);
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_LEFT, true, time);
        instance.updateWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_RIGHT, true, time);
        assertEquals(LocomotionState.Configuration.STRAIGHT, instance.getConfiguration());
    }

    @Test
    public void testPodPosConfigurations() throws Exception {
        Instant time = Instant.now();
        LocomotionState instance = new LocomotionState();
        // Start in STRAIGHT configuration
        assertEquals(LocomotionState.Configuration.STRAIGHT, instance.getConfiguration());
        // Enter STRAFE configuration
        instance.updateWheelPodPos(LocomotionState.Wheel.BACK_LEFT, 85f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.BACK_RIGHT, 85f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.FRONT_LEFT, 85f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.FRONT_RIGHT, 85f, time);
        assertEquals(LocomotionState.Configuration.STRAFE, instance.getConfiguration());
        // Enter TURN configuration
        instance.updateWheelPodPos(LocomotionState.Wheel.BACK_LEFT, 45f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.BACK_RIGHT, 45f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.FRONT_LEFT, 45f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.FRONT_RIGHT, 45f, time);
        assertEquals(LocomotionState.Configuration.TURN, instance.getConfiguration());
        // Enter STRAIGHT configuration
        instance.updateWheelPodPos(LocomotionState.Wheel.BACK_LEFT, 5f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.BACK_RIGHT, 5f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.FRONT_LEFT, 5f, time);
        instance.updateWheelPodPos(LocomotionState.Wheel.FRONT_RIGHT, 5f, time);
        assertEquals(LocomotionState.Configuration.STRAIGHT, instance.getConfiguration());
    }
}