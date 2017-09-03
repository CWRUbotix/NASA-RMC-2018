package com.cwrubotix.glennifer.robot_state;

import com.cwrubotix.glennifer.Messages;
import com.google.protobuf.InvalidProtocolBufferException;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.GetResponse;

import java.io.IOException;
import java.time.Instant;
import java.util.concurrent.TimeoutException;

import static org.junit.Assert.assertNotNull;

public class SubscriptionTestMain {

    public static void main(String[] args) {
        try {
            LocomotionState locomotionState;
            ExcavationState excavationState;
            DepositionState depositionState;
            StateModule module;

            locomotionState = new LocomotionState();
            excavationState = new ExcavationState();
            depositionState = new DepositionState();
            module = new StateModule(locomotionState, excavationState, depositionState, "amq.topic");
            module.start();

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
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (InvalidProtocolBufferException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (TimeoutException e) {
            e.printStackTrace();
        }
    }

    private static Messages.UnixTime instantToUnixTime(Instant time) {
        Messages.UnixTime.Builder unixTimeBuilder = Messages.UnixTime.newBuilder();
        unixTimeBuilder.setTimeInt(time.getEpochSecond());
        unixTimeBuilder.setTimeFrac(time.getNano() / 1000000000F);
        return unixTimeBuilder.build();
    }
}
