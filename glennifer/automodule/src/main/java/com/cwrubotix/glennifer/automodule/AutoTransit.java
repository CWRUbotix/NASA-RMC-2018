package com.cwrubotix.glennifer.automodule;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.ProgressReport;
import com.cwrubotix.glennifer.Messages.RpmUpdate;
import com.cwrubotix.glennifer.Messages.SpeedControlCommand;
import com.rabbitmq.client.*;

import java.io.IOException;
import java.time.Instant;
import java.util.concurrent.TimeoutException;

/**
 * Module for controlling movement
 *
 * @author Imran Hossain
 */
public class AutoTransit extends Module {
    private final Position DUMP_BIN = new Position(0.0F, 0.0F, Math.PI);
    /* Horizontal line representing where digging arena starts. */
    private final Position DIGGING_AREA = new Position(0.0F, 4.41F, -1.0);
    private final float CLEARANCE_DIST = 0.3F; // Setting this to 30cm for now.
					       // Will have to change it after
					       // testing locomotion.
    private final float TRAVEL_SPEED = 25.0F; // Duty cycle in unit of percent
    private static Position currentPos;
    private PathFinder pathFinder;
    private Path currentPath;
    private Position subTarget;
    private boolean launched = false;

    // Messaging stuff
    private String exchangeName;
    private Connection connection;
    private Channel channel;

    /////// MESSAGING

    /**
     * Consumer class for launch command
     */
    public class TransitLaunchConsumer extends DefaultConsumer {
	public TransitLaunchConsumer(Channel channel) {
	    super(channel);
	}

	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
		throws IOException {

	    Messages.LaunchTransit cmd = Messages.LaunchTransit.parseFrom(body);
	    // Get current position
	    Position currentPos = new Position(cmd.getCurXPos(), cmd.getCurYPos(), cmd.getCurHeading());

	    Position destinationPos = new Position(cmd.getDestXPos(), cmd.getDestYPos(), 0f);
	    if (!launched) {
		launched = true;
		launchNavigation(currentPos, destinationPos);
	    }
	}
    }

    public class TransitSoftStopConsumer extends DefaultConsumer {
	public TransitSoftStopConsumer(Channel channel) {
	    super(channel);
	}

	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
		throws IOException {
	    Messages.TransitSoftStop cmd = Messages.TransitSoftStop.parseFrom(body);
	    launched = false;// TODO: Implement soft stop handler
	}
    }

    public class TransitHardStopConsumer extends DefaultConsumer {
	public TransitHardStopConsumer(Channel channel) {
	    super(channel);
	}

	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
		throws IOException {
	    Messages.TransitHardStop cmd = Messages.TransitHardStop.parseFrom(body);
	    System.err.println("AutoTransit Received Hard Stop");
	    System.exit(1); // End the process
	}
    }

    public class TransitNewObstacleConsumer extends DefaultConsumer {
	public TransitNewObstacleConsumer(Channel channel) {
	    super(channel);
	}

	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
		throws IOException {
	    // Parse message
	    Messages.TransitNewObstacle cmd = Messages.TransitNewObstacle.parseFrom(body);

	    // Construct obstacle data
	    float obsXPos = cmd.getXPos();
	    float obsYPos = cmd.getXPos();
	    obsXPos += currentPos.getX(); // TODO Temporarily converting
					  // obstacle position relative to the
					  // tag
	    obsYPos += currentPos.getY(); // Probably good idea to move this to
					  // the StateModule in the future.
	    float obsDiameter = cmd.getDiameter();
	    Obstacle newObs = new Obstacle(obsXPos, obsYPos, obsDiameter);

	    launchNavigation(currentPos, newObs);
	}
    }

    public class LocalizationPositionConsumer extends DefaultConsumer {
	public LocalizationPositionConsumer(Channel channel) {
	    super(channel);
	}

	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body)
		throws IOException {
	    // parse message
	    Messages.LocalizationPosition pos = Messages.LocalizationPosition.parseFrom(body);

	    // Updates current position
	    currentPos = new Position(pos.getXPosition(), pos.getYPosition(), pos.getBearingAngle());
	    System.out.println("current pos:" + currentPos);
	    if (launched && currentPath.getPath().size() < 1) {
		launched = false;
		ProgressReport report = ProgressReport.newBuilder().setDone(true)
			.setTimestamp(instantToUnixTime(Instant.now())).build();
		this.getChannel().basicPublish(exchangeName, "progress.transit", null, report.toByteArray());
	    } else if (subTarget.equals(currentPos)) {
		moveToPos(subTarget = currentPath.getPath().remove(), currentPath.getPath().getFirst());
	    } else {
		moveToPos(currentPos, subTarget);
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

    public static Position getCurrentPos() {
	return currentPos;
    }

    private void launchNavigation(Position currentPos, Position destination) throws IOException {
	if (launched) {
	    AutoTransit.currentPos = currentPos;
	    pathFinder = new PathFinder(new ModifiedAStar(), currentPos, destination);
	    currentPath = pathFinder.getPath();
	    moveToPos(subTarget = currentPath.getPath().remove(), currentPath.getPath().getFirst());
	}
    }

    private void launchNavigation(Position currentPos, Obstacle obs) throws IOException {
	if (launched) {
	    pathFinder.setCurrentPos(currentPos);
	    try {
		pathFinder.registerObstacle(obs);
	    } catch (PathFinder.DestinationModified destinationModified) {
	    } finally {
		currentPath = pathFinder.getPath();
	    }
	    moveToPos(subTarget = currentPath.getPath().remove(), currentPath.getPath().getFirst());
	}
    }

    private void moveToPos(Position currPos, Position destPos) throws IOException {
	// Compute angle to turn to -- clockwise is positive
	double angleBetween = Position.angleBetween(currPos, destPos);
	if (launched) {
	    if(Math.abs(currentPos.getHeading() - angleBetween) < 0.05)
		turnAngle(angleBetween);
	    else
		driveTo(destPos);
	}
    }

    private void turnAngle(double angle) throws IOException {
	if (angle < 0)
	    return;

	/*
	 * Determine direction Tell robot to turn while we're not at correct
	 * heading sleep 100ms stop
	 */
	// Build messages
	SpeedControlCommand rWheelsMsg = SpeedControlCommand.newBuilder()
		.setRpm(-Math.signum((float) angle) * TRAVEL_SPEED).build();

	SpeedControlCommand lWheelsMsg = SpeedControlCommand.newBuilder()
		.setRpm(Math.signum((float) angle) * TRAVEL_SPEED).build();

	// Tell wheels to start moving
	this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.front_right.wheel_rpm", null,
		rWheelsMsg.toByteArray());
	this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.back_right.wheel_rpm", null,
		rWheelsMsg.toByteArray());
	this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.front_left.wheel_rpm", null,
		lWheelsMsg.toByteArray());
	this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.back_left.wheel_rpm", null,
		lWheelsMsg.toByteArray());

	System.out.println("Trying to turn...");
	// Stop when angle is reached
	while (!(Math.abs(currentPos.getHeading() - angle) < 0.05)) {
	    try {
		Thread.sleep(100);
	    } catch (InterruptedException e) {
		e.printStackTrace();
	    }
	}
	System.out.println("Desired angle reached");
    }

    private void driveTo(Position destPos) throws IOException {
	/*
	 * Time = distance / speed Tell robot to drive while
	 * !equalsWithinError(curr, dest, error) sleep 100ms stop
	 */

	// Drive
	SpeedControlCommand driveMsg = SpeedControlCommand.newBuilder().setRpm(TRAVEL_SPEED).build();

	this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.front_right.wheel_rpm", null,
		driveMsg.toByteArray());
	this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.back_right.wheel_rpm", null,
		driveMsg.toByteArray());
	this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.front_left.wheel_rpm", null,
		driveMsg.toByteArray());
	this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.back_left.wheel_rpm", null,
		driveMsg.toByteArray());
	System.out.println("Moving forward...");
	// Stop when destination reached (within tolerance)
	while (!Position.equalsWithinError(currentPos, destPos, 0.1)) {
	    try {
		Thread.sleep(100);
	    } catch (InterruptedException e) {
		e.printStackTrace();
	    }
	}
	System.out.println("Reached subgoal");
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

	queueName = channel.queueDeclare().getQueue();
	this.channel.queueBind(queueName, exchangeName, "loc.post");
	this.channel.basicConsume(queueName, true, new LocalizationPositionConsumer(channel));

	System.out.println("Ready to listen to messages");
    }

    public static void main(String[] args) {
	AutoTransit transitModule = new AutoTransit();
	transitModule.start();
    }

}
