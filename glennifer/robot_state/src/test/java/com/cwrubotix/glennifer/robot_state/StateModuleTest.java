package com.cwrubotix.glennifer.robot_state;

import com.cwrubotix.glennifer.Messages;

import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.GetResponse;

import java.io.IOException;
import java.time.Instant;
import java.util.concurrent.TimeoutException;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class StateModuleTest {

    private LocomotionState locomotionState; // TODO: mock this
    private ExcavationState excavationState;
    private DepositionState depositionState;
    private StateModule module;

    public StateModuleTest() { }

    private Messages.UnixTime instantToUnixTime(Instant time) {
        Messages.UnixTime.Builder unixTimeBuilder = Messages.UnixTime.newBuilder();
        unixTimeBuilder.setTimeInt(time.getEpochSecond());
        unixTimeBuilder.setTimeFrac(time.getNano() / 1000000000F);
        return unixTimeBuilder.build();
    }

    @Before
    public void setUp() throws InterruptedException {
        locomotionState = new LocomotionState();
        excavationState = new ExcavationState();
        depositionState = new DepositionState();
        module = new StateModule(locomotionState, excavationState, depositionState, "amq.topic");
        module.start();
    }

    @After
    public void tearDown() throws IOException, TimeoutException, InterruptedException {
        module.stop();
    }

    /**
     * Test of run method
     */
    /*
    @Test
    public void testRun() throws Exception {
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        Connection connection = factory.newConnection();
        Channel channel = connection.createChannel();

        Messages.RpmUpdate.Builder rpmUpdateFactory = Messages.RpmUpdate.newBuilder();
        rpmUpdateFactory.setRpm(42F);
        rpmUpdateFactory.setTimestamp(instantToUnixTime(Instant.now()));
        Messages.RpmUpdate message = rpmUpdateFactory.build();
        channel.basicPublish("amq.topic", "sensor.locomotion.back_left.wheel_rpm", null, message.toByteArray());

        channel.close();
        connection.close();

        Thread.sleep(1000);

        float result = locomotionState.getWheelRpm(LocomotionState.Wheel.BACK_LEFT);
        assertEquals(42F, result, 0);
    }
    */

    /**
     * Test subscription
     */
    /*
    @Test
    public void testSubscribe() throws Exception {
        Thread.sleep(1000);
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        Connection connection = factory.newConnection();
        Channel channel = connection.createChannel();

        // Create queue
        channel.exchangeDeclare("amq.topic", "topic", true);
        String queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, "amq.topic", queueName);

        // Queue is known to be empty

        // Send message
        Messages.StateSubscribe subMsg = Messages.StateSubscribe.newBuilder()
                .setStartTime(instantToUnixTime(Instant.now()))
                .setInterval(0.1f)
                .setReplyKey(queueName)
                .setLocomotionSummary(true)
                .setLocomotionDetailed(true)
                .setDepositionSummary(true)
                .setDepositionDetailed(true)
                .setExcavationSummary(true)
                .setExcavationDetailed(true)
                .build();

        channel.basicPublish("amq.topic", "state.subscribe", null, subMsg.toByteArray());

        // Wait
        Thread.sleep(1000);

        GetResponse response = channel.basicGet(queueName, true);
        assertNotNull("Failed to get message from state subscription", response);
        channel.close();
        connection.close();
        byte[] body = response.getBody();
        Messages.State s = Messages.State.parseFrom(body);
        System.out.println(s);
    }
    */
}
