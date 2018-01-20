package main.java.com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import com.rabbitmq.client.AMQP;

import main.java.com.cwrubotix.glennifer.Messages;
import main.java.com.cwrubotix.glennifer.Messages.LocomotionControlCommandStraight;
import main.java.com.cwrubotix.glennifer.Messages.SpeedContolCommand;
import main.java.com.cwrubotix.glennifer.Messages.Fault;
import main.java.com.cwrubotix.glennifer.Messages.UnixTime;

import java.io.IOException;
import java.time.Instant;
import java.time.Duration;
import java.util.concurrent.TimeoutException;

public class AutoDump {
    private float dumpAngle;
    private float load;

	/*
	 * TODO list
	 * 1) Subscribe to appropriate sensors (load cells, dumping angle)
	 * 2) Keep track of time we have.
	 * 3) Come up with possible errors and handling mechanism.
	 * 4) set up connection factory
	 * 
	 */

}
