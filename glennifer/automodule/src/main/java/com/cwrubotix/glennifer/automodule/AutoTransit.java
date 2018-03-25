package com.cwrubotix.glennifer.automodule;

import com.cwrubotix.glennifer.Messages;
import com.rabbitmq.client.*;

import java.io.IOException;
import java.util.concurrent.TimeoutException;

/**
 * Module for controlling movement
 *
 * @author Imran Hossain
 */
public class AutoTransit extends Module {
	private final Position DUMP_BIN = new Position(0.0F, 0.0F, Math.PI);
	/*Horizontal line representing where digging arena starts.*/
	private final Position DIGGING_AREA = new Position(0.0F, 4.41F, -1.0);
	private final float CLEARANCE_DIST = 0.3F; //Setting this to 30cm for now. Will have to change it after testing locomotion.
    private final float TRAVEL_SPEED = 1.0F; // Sensible speed at which to travel
	private static Position currentPos;
	private PathFinder pathFinder;

	// Messaging stuff
	private String exchangeName;
	private Connection connection;
	private Channel channel;


	/*
	 * TODO LIST
	 * 
	 * 1) Create Wrapper data type for coordinate values that we receive from localization
	 * 2) Subscribe to appropriate sensor values. (location within the arena, locomotion motors)
	 * 3) Come up with path planning algorithm.
	 * 4) Come up with possible errors and handling mechanism.
	 * 5) Set up the Connection Factory
	 * 
	 */

	/////// MESSAGING

	/**
	 * Consumer class for launch command
	 */
	public class TransitLaunchConsumer extends DefaultConsumer {
		public TransitLaunchConsumer(Channel channel) {
			super(channel);
		}

		@Override
		public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
			Messages.LaunchTransit cmd = Messages.LaunchTransit.parseFrom(body);
			// Get current position
			Position currentPos = new Position(
					cmd.getCurXPos(),
					cmd.getCurYPos(),
					cmd.getCurHeading());

			Position destinationPos = new Position(
					cmd.getDestXPos(),
					cmd.getDestYPos(),
					0f);

			pathFinder = new PathFinder(new ModifiedAStar(), currentPos, destinationPos);
			Path currentPath = pathFinder.getPath();

			/* TODO Translate Path positions to HCI -- This possibly should be run in a loop on a separate method
				1. Create delta between first and next position in path
				2. Work with Steven to translate that to a message
				3. Construct message to send to HCI
			 */
			moveToPos(currentPath.getPoint(0), currentPath.getPoint(1));

        }
	}

	public class TransitSoftStopConsumer extends DefaultConsumer {
		public TransitSoftStopConsumer(Channel channel) {
			super(channel);
		}

		@Override
		public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
			Messages.TransitSoftStop cmd = Messages.TransitSoftStop.parseFrom(body);
			// TODO: Implement soft stop handler
		}
	}

	public class TransitHardStopConsumer extends DefaultConsumer {
		public TransitHardStopConsumer(Channel channel) {
			super(channel);
		}

		@Override
		public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
		    Messages.TransitHardStop cmd = Messages.TransitHardStop.parseFrom(body);
			// TODO: Implement hard stop handler
		}
	}

	public class TransitNewObstacleConsumer extends DefaultConsumer {
		public TransitNewObstacleConsumer(Channel channel) {
			super(channel);
		}

		@Override
		public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
			// Parse message
			Messages.TransitNewObstacle cmd = Messages.TransitNewObstacle.parseFrom(body);

			// Construct obstacle data
			float obsXPos = cmd.getXPos();
			float obsYPos = cmd.getXPos();
			float obsDiameter = cmd.getDiameter();
			Obstacle newObs = new Obstacle(obsXPos, obsYPos, obsDiameter);

			pathFinder.registerObstacle(newObs);
		}
	}

	/////// MODULE LOGIC

	public AutoTransit() {
		this("amq.topic");
	}

	public AutoTransit(String exchangeName) {
		this.exchangeName = exchangeName;
	}

	public static Position getCurrentPos(){
		return currentPos;
	}

	private void moveToPos(Position currPos, Position destPos) {
		// Compute angle to turn to -- clockwise is positive
        double angleBetween = Position.angleBetween(currPos, destPos);

		turnAngle(angleBetween);
		driveTo(destPos);
	}

	private void turnAngle(double angle) {
        /*
        Determine direction
        Tell robot to turn
        while we're not at correct heading
            sleep 100ms
        stop
         */
        if (angle > 0) {
            // right front + back go backwards
            // left front + back go forwards
            // use RpmUpdate to tell motors to move
        } else if (angle < 0) {
            // opposite
        }
    }

    private void driveTo(Position destPos) {
	    /*
	    Time = distance / speed
	    Tell robot to drive
	    while !equalsWithinError(curr, dest, error)
	        sleep 100ms
        stop
	     */
    }

    @Override
    protected void runWithExceptions() throws IOException, TimeoutException {
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        this.connection = factory.newConnection();
        this.channel = connection.createChannel();

        // Listeners for commands
		String queueName = channel.queueDeclare().getQueue();
		this.channel.queueBind(queueName, exchangeName, "launch.transit");
		this.channel.basicConsume(queueName, true, new TransitLaunchConsumer(channel));

		queueName = channel.queueDeclare().getQueue();
		this.channel.queueBind(queueName, exchangeName, "softstop.transit");
		this.channel.basicConsume(queueName, true, new TransitSoftStopConsumer(channel));

		queueName = channel.queueDeclare().getQueue();
		this.channel.queueBind(queueName, exchangeName, "hardstop.transit");
		this.channel.basicConsume(queueName, true, new TransitHardStopConsumer(channel));

		queueName = channel.queueDeclare().getQueue();
		this.channel.queueBind(queueName, exchangeName, "newobstacle.transit");
		this.channel.basicConsume(queueName, true, new TransitNewObstacleConsumer(channel));

		// TODO Move to position[1], remove position[0], repeat
    }

}
