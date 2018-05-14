package com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;

import java.io.*;
import java.time.LocalDateTime;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.TimeoutException;

import com.cwrubotix.glennifer.Messages;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;


/**
 * Module to collect data and write it to a file
 *
 * @author Imran Hossain
 * @author Seohyun Jung
 */
public class LogModule extends Module {

    /// Robot-specific fields
    PathFinder pathFinder;

    /// Logging fields
    private File pathLogFile, driveLogFile;
    private Writer pathLogWriter, driveLogWriter;

    public LogModule() {
        this("amq.topic");
    }

    public LogModule(String exchangeName) {
        this.exchangeName = exchangeName;
    }


    @Override
    protected void runWithExceptions() throws IOException, TimeoutException {
        Runtime runtime = Runtime.getRuntime();
        runtime.addShutdownHook(new Thread() {
            public void run() {
                LogModule.this.stop();
            }
        });

        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        this.connection = factory.newConnection();
        this.channel = connection.createChannel();

        // Create log files and writers
        LocalDateTime openTime = LocalDateTime.now();
        pathLogFile = new File(String.format("%d-%d-%d-%d:%d_Path.txt",
                openTime.getYear(), openTime.getMonthValue(), openTime.getDayOfMonth(),
                openTime.getHour(), openTime.getMinute()));
        pathLogWriter = new BufferedWriter(new FileWriter(pathLogFile));
        driveLogFile = new File(String.format("%d-%d-%d-%d:%d_Drive.txt",
                openTime.getYear(), openTime.getMonthValue(), openTime.getDayOfMonth(),
                openTime.getHour(), openTime.getMinute()));
        driveLogWriter = new BufferedWriter(new FileWriter(driveLogFile));

        String queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "motor.#");
        channel.basicConsume(queueName, true, new MotorConsumer());

        // Subscribing to StateModule
        Messages.StateSubscribe msg = Messages.StateSubscribe.newBuilder().setReplyKey("logModule")
                .setInterval(0.1F)
                .setDepositionDetailed(true)
                .setDepositionSummary(true)
                .setExcavationDetailed(true)
                .setExcavationSummary(true)
                .setLocomotionDetailed(true)
                .setLocomotionSummary(true)
                .setLocObsDetailed(true)
                .build();
        this.channel.basicPublish(exchangeName, "state.subscribe", null, msg.toByteArray());

        queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "logModule");
        channel.basicConsume(queueName, true, new SensorConsumer());
    }

    @Override
    public void stop() {
        // Subscribing to StateModule
        Messages.StateSubscribe msg = Messages.StateSubscribe.newBuilder().setReplyKey("logModule")
                .setInterval(0.1F)
                .setDepositionDetailed(true)
                .setDepositionSummary(true)
                .setExcavationDetailed(true)
                .setExcavationSummary(true)
                .setLocomotionDetailed(true)
                .setLocomotionSummary(true)
                .setLocObsDetailed(true)
                .build();
        try {
            this.channel.basicPublish(exchangeName, "state.unsubscribe", null, msg.toByteArray());
        } catch (IOException e) {
            System.err.println("Failed to unsubscribe from StateModule");
            e.printStackTrace();
        }

        //Finish Logging to Files
        try {
            pathLogWriter.close();
            driveLogWriter.close();
        } catch (IOException e) {
            System.err.println("Something went wrong while trying to close file write streams");
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        LogModule logModule = new LogModule();
        logModule.start();
    }

    private class MotorConsumer extends DefaultConsumer {
        public MotorConsumer() {
            super(channel);
        }

        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {

        }
    }

    private class SensorConsumer extends DefaultConsumer {
        public SensorConsumer() {
            super(channel);
        }

        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {

        }
    }
}
