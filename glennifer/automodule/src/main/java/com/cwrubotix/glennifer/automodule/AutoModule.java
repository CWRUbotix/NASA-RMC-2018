package com.cwrubotix.glennifer.automodule;

import com.cwrubotix.glennifer.Messages.LaunchTransit;
import com.cwrubotix.glennifer.Messages.TransitSoftStop;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;

import java.io.IOException;
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

		// Tell transit to start for N minutes
		LaunchTransit msg1 = LaunchTransit.newBuilder()
				// TODO set message properties (destination, current position, etc)
				.setTimeAlloc(180)
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
}
