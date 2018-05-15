package com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import com.rabbitmq.client.AMQP;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.SpeedControlCommand;

import java.io.IOException;
import java.util.concurrent.TimeoutException;

/**
 * Module that controls digging cycle of the run
 *
 * @author Seohyun Jung
 * @author Michael Schaffer
 */
public class AutoDrillModule extends Module {

    private final float currentUpperLimit = 7500.0F;
    private final float currentLowerLimit = 5000.0F;
    private float transSpeed = 8;
    private float stallSpeed = -20;

    private enum DrillJob {DEEP, NONE}

    private DrillJob currentJob = DrillJob.NONE;
    private DrillJob lastJob = DrillJob.NONE;

    private boolean isStalled = false;
    private float bc_current = 0.0F;

    /**
     * Depending on current task and current the excavation motor is pulling,
     * this method sends out appropriate motor control messages.
     */
    private void updateMotors() {

	try {
	    switch (currentJob) {
	    case NONE:
		if (currentJob != lastJob) {
		    excavationTranslationControl(0);
		    Thread.sleep(10000L);
		    excavationConveyorRPM(0);
		    excavationTranslationControl(-30);
		}
		break;
	    case DEEP:
		if (isStalled) {
		    System.out.println("stalled");
		    excavationConveyorRPM(100);
		    excavationTranslationControl(stallSpeed);
		} else {
		    System.out.println("not stalled");
		    excavationConveyorRPM(100);
		    excavationTranslationControl(transSpeed);
		}
		break;
	    }
	    lastJob = currentJob;
	} catch (IOException e) {
	    System.err.println("AutoDrill failed to publish message with exception:");
	    e.printStackTrace();
	} catch (InterruptedException e) {
	    System.err.println("Something went wrong while trying to empty buckets before ending digging");
	    e.printStackTrace();
	}
    }

    /**
     * This method compares current the motor is pulling and determine whether
     * the motor is in stall
     */
    private void detectStall() {
	if (!isStalled && bc_current > currentUpperLimit) {
	    // Transition to stalled
	    System.out.println("Excavation System overloaded, starts retracting");
	    isStalled = true;
	} else if (isStalled && bc_current <= currentLowerLimit) {
	    // Transition to unstalled
	    isStalled = false;
	    System.out.println("Current back to normal, resuming normal digging");

	}
    }

    /**
     * Wrapper method for creating excavation translation command message
     *
     * @param targetValue the amount of translation we want
     * @throws IOException when failed to create/publish message
     */
    private void excavationTranslationControl(float targetValue) throws IOException {
	Messages.SpeedControlCommand spc = Messages.SpeedControlCommand.newBuilder().setRpm(targetValue).setTimeout(123)
		.build();
	this.channel.basicPublish(exchangeName, "motorcontrol.excavation.conveyor_translation_speed", null,
		spc.toByteArray());
    }

    /**
     * Wrapper method for creating excavation conveyor RPM command message
     *
     * @param targetValue conveyor RPM that we want
     * @throws IOException when failed to create/publish message
     */
    private void excavationConveyorRPM(float targetValue) throws IOException {
	Messages.SpeedControlCommand msg = SpeedControlCommand.newBuilder().setRpm(targetValue).setTimeout(123).build();
	AutoDrillModule.this.channel.basicPublish(exchangeName, "motorcontrol.excavation.bucket_conveyor_rpm", null,
		msg.toByteArray());
    }

    public AutoDrillModule() {
	this("amq.topic");
    }

    public AutoDrillModule(String exchangeName) {
	this.exchangeName = exchangeName;
    }

    /**
     * The method where everything gets set up to operate.
     ** 
     * @throws IOException
     * @throws TimeoutException
     */
    public void runWithExceptions() throws IOException, TimeoutException {

	// Setup connection
	ConnectionFactory factory = new ConnectionFactory();
	factory.setHost("localhost");
	this.connection = factory.newConnection();
	this.channel = connection.createChannel();

	// Listen for DrillDeep command
	String queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "subsyscommand.excavation.dig_deep");
	this.channel.basicConsume(queueName, true, new DrillDeepConsumer(channel));

	// Listen for DrillEnd command
	queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "subsyscommand.excavation.dig_end");
	this.channel.basicConsume(queueName, true, new DrillEndConsumer(channel));

	// Subscribing to StateModule
	Messages.StateSubscribe msg = Messages.StateSubscribe.newBuilder().setReplyKey("autoDrillModule")
		.setInterval(0.1F).setDepositionDetailed(false).setDepositionSummary(true).setExcavationDetailed(true)
		.setExcavationSummary(false).setLocomotionDetailed(true).setLocomotionSummary(false)
		.setLocObsDetailed(true).build();
	this.channel.basicPublish(exchangeName, "state.subscribe", null, msg.toByteArray());

	// Listen for state update
	queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "autoDrillModule");
	this.channel.basicConsume(queueName, true, new StateUpdateConsumer(channel));
	System.out.println("Waitng commands...");
    }

    @Override
    public void stop() {
	try {
	    // Unsubscribe from StateModule
	    Messages.StateSubscribe msg = Messages.StateSubscribe.newBuilder().setReplyKey("autoDrillModule")
		    .setInterval(0.2F).setDepositionDetailed(false).setDepositionSummary(true)
		    .setExcavationDetailed(true).setExcavationSummary(false).setLocomotionDetailed(true)
		    .setLocomotionSummary(false).setLocObsDetailed(true).build();
	    this.channel.basicPublish(exchangeName, "state.unsubscribe", null, msg.toByteArray());
	    channel.close();
	    connection.close();
	} catch (TimeoutException | IOException e) {
	    // Do nothing
	}
    }

    public static void main(String[] args) {
	AutoDrillModule module = new AutoDrillModule();
	module.start();
    }

    /**
     * Consumer class for DrillDeepCommand.
     */
    private class DrillDeepConsumer extends DefaultConsumer {
	public DrillDeepConsumer(Channel channel) {
	    super(channel);
	}

	/**
	 * Takes in message queue from RabbitMQ, interprets the message, and
	 * then starts drill deep command. handleDelivery method is called when
	 * the channel Object calls basicConsume method with DrillDeepConsumer
	 * instance.
	 *
	 * @param consumerTag you can disregard this input. RabbitAMQP stuff
	 * @param envelope you can disregard this input. RabbitAMQP stuff
	 * @param properties will always be null for our purposes
	 * @param body the byte array representation of the incoming message itself
	 * @throws IOException when message system messes up with us
	 */
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
		throws IOException {
	    // Transition to dig deep
	    currentJob = DrillJob.DEEP;
	    // excavationAngleControl(bc_angle);

	    // Parse the incoming message to get target depth and speed we want
	    detectStall();
	    updateMotors(); // Starts digging with given goals.
	}
    }

    /**
     * Consumer class for DrillEndCommand
     */
    private class DrillEndConsumer extends DefaultConsumer {
	public DrillEndConsumer(Channel channel) {
	    super(channel);
	}

	/**
	 * Takes in message queue from RabbitMQ, interprets the message, and
	 * then starts drill end command. handleDelivery method is called when
	 * the channel Object calls basicConsume method with DrillEndConsumer
	 * instance.
	 *
	 * @param consumerTag you can disregard this input. RabbitAMQP stuff
	 * @param envelope you can disregard this input. RabbitAMQP stuff
	 * @param properties will always be null for our purposes
	 * @param body the byte array representation of the incoming message itself
	 * @throws IOException when message system messes up with us
	 */
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
		throws IOException {
	    currentJob = DrillJob.NONE; // Transition to DrillJob.NONE state to
					// end digging cycle
	    updateMotors(); // Ends digging cycle
	}
    }

    private class StateUpdateConsumer extends DefaultConsumer {
	public StateUpdateConsumer(Channel channel) {
	    super(channel);
	}

	@Override
	public void handleDelivery(String conumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
		throws IOException {
	    Messages.State msg = Messages.State.parseFrom(body);
	    bc_current = msg.getExcDetailed().getCurrent();
	    detectStall();
	    updateMotors();
	}
    }

}
