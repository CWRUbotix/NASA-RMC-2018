package main.java.com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import com.rabbitmq.client.AMQP;

import com.cwrubotix.glennifer.Messages;

import java.io.IOException;
import java.time.Instant;
import java.time.Duration;
import java.util.concurrent.TimeoutException;



public class AutoTransit extends Module {
	private final Position DUMP_BIN = new Position(0.0F, 0.0F, Math.PI);
	/*Horizontal line representing where digging arena starts.*/
	private final Position DIGGING_AREA = new Position(0.0F, 4.41F, -1.0);
	private final float CLEARANCE_DIST = 0.3F; //Setting this to 30cm for now. Will have to change it after testing locomotion.
	private static Position currentPos;	
	
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
