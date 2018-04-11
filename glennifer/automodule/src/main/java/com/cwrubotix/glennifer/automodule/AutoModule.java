package com.cwrubotix.glennifer.automodule;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.LaunchTransit;
import com.cwrubotix.glennifer.Messages.TransitSoftStop;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import java.io.IOException;
import java.time.Instant;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeoutException;

/**
 * 
 * Autonomous Module which controls robot's movements through mining cycles.
 * AutoModule is a state function which switches back and forth between Transit state,
 * Digging state, Dumping state. AutoModule should be able to decide robot's current/next actions,
 * depending on which state it is in.
 * 
 * @author Seohyun Jung
 * @author Imran Hossain
 *
 */
public class AutoModule extends Module {
	private Stage currentStage;
	private enum Stage {TRANSIT, DIGGING, DUMPING, EMERGENCY};

	private String exchangeName;
	private Connection connection;
	private Channel channel;
	private Position startPos;
	private boolean tagFound = false;
	
	public class LocalizationPositionConsumer extends DefaultConsumer {
	    public LocalizationPositionConsumer(Channel channel){
		super(channel);
	    }
	    
	    @Override
	    public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
		//parse message
		Messages.LocalizationPosition pos = Messages.LocalizationPosition.parseFrom(body);
		
		// Updates current position
		startPos = new Position(pos.getXPosition(), pos.getYPosition(), pos.getBearingAngle());
		tagFound = true;
		System.out.println("starting pos: " + startPos);
	    }
	}
	
	/*
	 *TODO list
	 *	1) Decide on course of actions on possible situations (Obstacles, Path plan, Setting up Dumping position)
	 *	2) Create appropriate Consumers, Messages system for decision making messages.
	 *	3) Subscribe for sensor values or modules needed.
	 *	4) Come up with possible errors and handling mechanism.
	 *	5) Set up Connection Factory.
	 */

	@Override
	protected void runWithExceptions() throws IOException, TimeoutException {
        // Setup connection
		ConnectionFactory factory = new ConnectionFactory();
		factory.setHost("localhost");
		this.connection = factory.newConnection();
		this.channel = connection.createChannel();

		// TODO AutoModule needs to turn around and scan for AprilTags

		// Setup timer for timing tasks
		Timer taskTimer = new Timer("Task Timer");
		while(!tagFound){
		    try{
		    Thread.sleep(100);
		    } catch(InterruptedException e){
			e.printStackTrace();
		    }
		}
		// Tell transit to start for N minutes
		LaunchTransit msg1 = LaunchTransit.newBuilder()
				.setCurXPos(startPos.getX())
				.setCurYPos(startPos.getY())
				.setCurHeading((float) startPos.getHeading())
				.setDestXPos(0.0F)
				.setDestYPos(3.25F)
				.setTimeAlloc(180)
				.setTimestamp(instantToUnixTime(Instant.now()))
				.build();
		this.channel.basicPublish(exchangeName, "launch.transit", null, msg1.toByteArray());

		TransitSoftStop msg2 = TransitSoftStop.newBuilder()
				.build();
		taskTimer.schedule(new TimerTask() {
			@Override
			public void run() {
				try {
					AutoModule.this.channel.basicPublish(exchangeName, "softstop.transit", null, msg2.toByteArray());
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}, 1800000);
	}

    public static void main(String[] args) {
		AutoModule autoModule = new AutoModule();
		autoModule.start();
    }
}
