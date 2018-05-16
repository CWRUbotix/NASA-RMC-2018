package com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;

import java.io.*;
import java.time.Duration;
import java.time.LocalDateTime;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.TimeoutException;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.ObstaclePosition;
import com.cwrubotix.glennifer.automodule.PathFinder.DestinationModified;
import com.cwrubotix.glennifer.automodule.PathFindingAlgorithm.AlgorithmFailureException;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import java.time.Instant;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Stack;

/**
 * Module to collect data and write it to a file
 *
 * @author Imran Hossain
 * @author Seohyun Jung
 */
public class LogModule extends Module {

    enum LogType {PATH, DRIVE}

    /// Robot-specific fields
    private PathFinder finder;
    /*Assuming we are doing two runs*/
    private Stack<Position> pos = new Stack<>();
    private Position currentDes;

    /// Logging fields
    private File pathLogFile, driveLogFile;
    private Writer pathLogWriter, driveLogWriter;

    /// Consumer fields
    private enum Drive {
        STRAIGHT, TURN, STOP
    }

    private enum Wheel {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT}

    private Drive currentDrive;
    private EnumMap<Wheel, Float> motorValues = new EnumMap<>(Wheel.class);
    private Instant lastStamp;
    private Position currentPos;
    private Position lastStartPos;
    private ArrayList<Obstacle> obstacles;

    public LogModule() {
        this("amq.topic");
    }

    public LogModule(String exchangeName) {
        this.exchangeName = exchangeName;
        pos.push(new Position(0.0, 1.0));
        pos.push(new Position(1.0, 6.0));
        pos.push(new Position(0.0, 1.0));
        pos.push(new Position(1.0, 6.0));
    }

    @Override
    protected void runWithExceptions() throws IOException, TimeoutException {
        Runtime runtime = Runtime.getRuntime();
        runtime.addShutdownHook(new Thread(() -> {
            System.out.println("Stopping gracefully...");
            LogModule.this.stop();
        }));

        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        this.connection = factory.newConnection();
        this.channel = connection.createChannel();

        // Create log files and writers
        LocalDateTime openTime = LocalDateTime.now();
        String openTimeString = openTime.format(DateTimeFormatter.ofPattern("yyyy-MM-ddTHH-mm"));

        pathLogFile = new File(String.format("%s_Path.txt", openTimeString));
        pathLogWriter = new BufferedWriter(new FileWriter(pathLogFile));
        driveLogFile = new File(String.format("%s_Drive.txt", openTimeString));
        driveLogWriter = new BufferedWriter(new FileWriter(driveLogFile));

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

        String queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "logModule");
        channel.basicConsume(queueName, true, new SensorConsumer());
    }

    /**
     * Stop the module gracefully, letting AMQ connections close and file writers flush to their files.
     */
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

    /**
     * Log a message to the appropriate file.
     *
     * @param type    The type of message to log
     * @param message Content of the message
     */
    private void log(LogType type, String message) {
        LocalDateTime logTime = LocalDateTime.now();
        String time = logTime.format(DateTimeFormatter.ofPattern("HH:mm:ss.SSS"));
        message = String.format("[%s] %s", time, message);
        try {
            switch (type) {
                case PATH:
                    pathLogWriter.write(message);
                    break;
                case DRIVE:
                    driveLogWriter.write(message);
                    break;
            }
        } catch (IOException e) {
            System.out.println("Something went wrong when trying to log data.");
        }
    }

    private Drive findDriveType() {
        float fl = motorValues.get(Wheel.FRONT_LEFT);
        float fr = motorValues.get(Wheel.FRONT_RIGHT);

        if (fl * fr > 0)
            return Drive.STRAIGHT;
        else if (fl * fr < 0)
            return Drive.TURN;
        else
            return Drive.STOP;
    }

    public static void main(String[] args) {
        LogModule logModule = new LogModule();
        logModule.start();
    }

    private class SensorConsumer extends DefaultConsumer {
        public SensorConsumer() {
            super(channel);
        }

        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            Messages.State msg = Messages.State.parseFrom(body);
            Messages.LocomotionStateDetailed lsd = msg.getLocDetailed();
            Messages.LocObsStateDetailed lod = msg.getLocObsDetailed();

            // Dealing with Localization Obstacle data first
            Position pos = new Position(lod.getLocPosition().getXPosition(), lod.getLocPosition().getYPosition(), lod.getLocPosition().getBearingAngle());

            if(!pos.equals(new Position(0,0,0))){ //when position info is not dummy info
        	if(currentPos == null){
        	    currentPos = pos;
        	    currentDes = LogModule.this.pos.pop();
        	    finder = new PathFinder(currentPos, currentDes);
        	    log(LogType.PATH, "Heading to position:" + currentDes.toString() + "\n" + finder.getPath().toString());
        	}else if(currentPos.equals(currentDes)){
        	    currentPos = pos;
        	    currentDes = LogModule.this.pos.pop();
        	    finder.recalculatePath(currentPos, currentDes);
        	    log(LogType.PATH, "Modified path at " + currentPos.toString() + "\nNow heading to : " + 
        		    currentDes.toString() + "\n" + finder.getPath());
        	}else if(currentPos.equals(finder.getPath().getPath().peekFirst())){
        	    currentPos = pos;
        	    finder.getPath().getPath().pop();
        	}else if(Math.abs(currentPos.getHeading() - finder.getPath().getPoint(0).getHeadingTo(finder.getPath().getPoint(0))) > 0.07){
        	    currentPos = pos;
        	    finder.recalculatePath(currentPos, currentDes);
        	}else{
        	    currentPos = pos;
        	}
            }
            
            /*Obstacle*/
            List<ObstaclePosition> obs = lod.getObstaclesList();
            for(ObstaclePosition op : obs){
        	Obstacle temp = new Obstacle(op.getXPosition(), op.getYPosition(), 0.15);
        	if(!obstacles.contains(temp)){
        	    obstacles.add(temp);
        	    try {
			finder.registerObstacle(temp);
		    } catch (AlgorithmFailureException | DestinationModified e) {
		    }
        	    log(LogType.PATH, "Obstacle registered at" + currentPos.toString() + "\nModified path:\n" + finder.getPath().toString());
        	}
            }
            
            if(Math.abs(currentPos.getHeading() - currentPos.getHeadingTo(finder.getPath().getPoint(0))) > Math.PI/7){
        	Messages.AutonomyNextHeading anh = Messages.AutonomyNextHeading.newBuilder()
        								       .setHeading((float) currentPos.getHeadingTo(finder.getPath().getPoint(0)))
        								       .build();
        	channel.basicPublish(exchangeName, "autonomy.next_heading", null, anh.toByteArray());
            }

            // Update motor values
            motorValues.put(Wheel.FRONT_LEFT, lsd.getFrontLeftRpm());
            motorValues.put(Wheel.FRONT_RIGHT, lsd.getFrontRightRpm());
            motorValues.put(Wheel.BACK_LEFT, lsd.getBackLeftRpm());
            motorValues.put(Wheel.BACK_RIGHT, lsd.getBackRightRpm());

            // Log or update drive type if necessary
            switch (findDriveType()) {
                case STOP:
                    // Log drive values
                    Duration driveTime = Duration.between(lastStamp, Instant.now());
                    log(LogType.DRIVE, String.format("%s from %s at %f rad to %s at %f rad in %d second(s)",
                            currentDrive == Drive.STRAIGHT ? "Drove" : "Turned",
                            lastStartPos, lastStartPos.getHeading(), currentPos, currentPos.getHeading(),
                            driveTime.getSeconds()));
                    lastStamp = Instant.now();
                    lastStartPos = currentPos;
                    break;
                case STRAIGHT:
                case TURN:
                    currentDrive = findDriveType();
                    break;
            }
        }
    }
}
