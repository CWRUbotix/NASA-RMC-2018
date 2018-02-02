package com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

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
public class AutoTransit{
	private final Position DUMP_BIN = new Position(0.0F, 0.0F, Math.PI, 0.0F);
	/*Horizontal line representing where digging arena starts.*/
	private final Position DIGGING_AREA = new Position(0.0F, 4.41F, -1.0, 0.0F);
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
		    // TODO: Implement launch handler
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

	//amqp stuff
	private UnixTime instantToUnixTime(Instant time) {
		UnixTime.Builder unixTimeBuilder = UnixTime.newBuilder();
		unixTimeBuilder.setTimeInt(time.getEpochSecond());
		unixTimeBuilder.setTimeFrac(time.getNano() / 1000000000F);
		return unixTimeBuilder.build();
	}

	//amqp stuff
	private void sendFault(int faultCode, Instant time) throws IOException {
		Fault.Builder faultBuilder = Fault.newBuilder();
		faultBuilder.setFaultCode(faultCode);
		faultBuilder.setTimestamp(instantToUnixTime(time));
		Fault message = faultBuilder.build();
		channel.basicPublish(exchangeName, "fault", null, message.toByteArray());
	}

	/////// START/STOP LOGIC

	private void runWithExceptions() throws IOException, TimeoutException {
		// Setup connection
		ConnectionFactory factory = new ConnectionFactory();
		factory.setHost("localhost");
		this.connection = factory.newConnection();
		this.channel = connection.createChannel();

		// Listener for launch command
		String queueName = channel.queueDeclare().getQueue();
		channel.queueBind(queueName, exchangeName, "launch.transit");
		this.channel.basicConsume(queueName, true, new TransitLaunchConsumer(channel));

		queueName = channel.queueDeclare().getQueue();
		channel.queueBind(queueName, exchangeName, "softstop.transit");
		channel.basicConsume(queueName, true, new TransitSoftStopConsumer(channel));

		queueName = channel.queueDeclare().getQueue();
		channel.queueBind(queueName, exchangeName, "hardstop.transit");
		channel.basicConsume(queueName, true, new TransitHardStopConsumer(channel));

		queueName = channel.queueDeclare().getQueue();
		channel.queueBind(queueName, exchangeName, "newobstacle.transit");
		channel.basicConsume(queueName, true, new TransitNewObstacleConsumer(channel));
	}

	public void start() {
		try {
			runWithExceptions();
		} catch (Exception e) {
		    try {
		    	sendFault(999, Instant.now());
			} catch (IOException e1) {
		    	e.printStackTrace();
		    	System.out.println(e.getMessage());
			}
		}
	}

	public void stop() {
		try {
			channel.close();
			connection.close();
		} catch (IOException | TimeoutException e) {
			// Do nothing
		}
	}
	
}
