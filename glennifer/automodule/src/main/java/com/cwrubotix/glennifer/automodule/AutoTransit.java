package main.java.com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import main.java.com.cwrubotix.glennifer.automodule.PathFinder.DestinationModified;
import main.java.com.cwrubotix.glennifer.automodule.PathFindingAlgorithm.AlgorithmFailureException;

import com.rabbitmq.client.AMQP;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.Fault;
import com.cwrubotix.glennifer.Messages.UnixTime;

import java.io.IOException;
import java.time.Instant;
import java.time.Duration;
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
			Position currentPos = new RobotPosition(
					cmd.getCurXPos(),
					cmd.getCurYPos(),
					cmd.getCurHeading(),
					0f);

			Position destinationPos = new RobotPosition(
					cmd.getDestXPos(),
					cmd.getDestYPos(),
					0f, 0f);

			// TODO Construct pathFinder when algorithms available
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

			try {
			    pathFinder.registerObstacle(newObs);
			} catch (AlgorithmFailureException | DestinationModified e) {
			    // TODO Auto-generated catch block
			    e.printStackTrace();
			}
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

    @Override
    protected void runWithExceptions() throws IOException, TimeoutException {
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        this.connection = factory.newConnection();
        this.channel = connection.createChannel();
        // Listen for commands...
    }

}
