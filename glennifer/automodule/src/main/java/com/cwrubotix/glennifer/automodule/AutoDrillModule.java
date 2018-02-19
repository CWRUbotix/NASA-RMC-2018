package main.java.com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import com.rabbitmq.client.AMQP;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.LocomotionControlCommandStraight;
import com.cwrubotix.glennifer.Messages.SpeedContolCommand;
import com.cwrubotix.glennifer.Messages.Fault;
import com.cwrubotix.glennifer.Messages.UnixTime;

import java.io.IOException;
import java.time.Instant;
import java.time.Duration;
import java.util.concurrent.TimeoutException;

/**
 * Module that controls digging cycle of the run
 *
 * @author Seohyun Jung
 * @author Michael Schaffer
 */
public class AutoDrillModule extends Module {
	/*
	 * TODO list for NASA RMC 2018
	 * 
	 * 1) Keep track of depth dug, load on the hopper.
	 * 2) Remove drill surface
	 * 3) Come up with more errors we want to deal with inside drill module.
	 * 
	 */

    /**
     * Upper limit of the current excavation motor is pulling under normal operation
     */
    private float currentUpperLimit = 10.0F;
    /**
     * Lower limit of the current excavation motor is pulling under normal operation
     */
    private float currentLowerLimit = 8.0F;

    /**
     * Consumer class for DrillDeepCommand.
     */
    private class DrillDeepConsumer extends DefaultConsumer {
        public DrillDeepConsumer(Channel channel) {
            super(channel);
        }

        /**
         * Takes in message queue from RabbitMQ, interprets the message, and then starts drill deep command.
         * handleDelivery method is called when the channel Object calls basicConsume method with DrillDeepConsumer instance.
         *
         * @param consumerTag you can disregard this input. RabbitAMQP stuff
         * @param envelope    you can disregard this input. RabbitAMQP stuff
         * @param properties  will always be null for our purposes
         * @param body        the byte array representation of the incoming message itself
         * @throws IOException when message system messes up with us
         */
        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            // Transition to dig deep
            currentJob = DrillJob.DEEP;
            // Parse the incoming message to get target depth and speed we want
			/*
			 * Note for NASA_RMC_2018, This message is intended for manual control to send to AutoDrillModule.
			 * We could either keep this message format and make AutoDrill to figure out DigSpeed we want and send message to itself
			 * Or keep this just for Manual Control and create make AutoDrill make decision on its own without using messages.
			 */
            Messages.ExcavationControlCommandDigDeep cmd = Messages.ExcavationControlCommandDigDeep.parseFrom(body);
            targetDepth = cmd.getDepth(); //How deep we want to dig
            digSpeed = cmd.getDigSpeed(); //How fast we want to dig
            modeStartTime = Instant.now(); //Time stamp of when this message was received.
            updateMotors(); //Starts digging with given goals.
        }
    }

    /**
     * Consumer class for DrillSurfaceCommand
     */
    private class DrillSurfaceConsumer extends DefaultConsumer {
        public DrillSurfaceConsumer(Channel channel) {
            super(channel);
        }

        /**
         * Takes in message queue from RabbitMQ, interprets the message, and then starts drill surface command.
         * handleDelivery method is called when the channel Object calls basicConsume method with DrillSurfaceConsumer instance.
         *
         * @param consumerTag you can disregard this input. RabbitAMQP stuff
         * @param envelope    you can disregard this input. RabbitAMQP stuff
         * @param properties  will always be null for our purposes
         * @param body        the byte array representation of the incoming message itself
         * @throws IOException when message system messes up with us
         */
        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            //Transition to dig as going forward
            currentJob = DrillJob.SURFACE;
            // Parse the incoming message to get target depth and speed we want
			/*
			 * Note for NASA_RMC_2018, This message is intended for manual control to send to AutoDrillModule.
			 * We could either keep this message format and make AutoDrill to figure out motor speeds we want and send message to itself
			 * Or keep this just for Manual Control and create make AutoDrill make decision on its own without using messages.
			 */
            Messages.ExcavationControlCommandDigSurface cmd = Messages.ExcavationControlCommandDigSurface.parseFrom(body);
            targetDepth = cmd.getDepth(); //How deep we want to dig
            digSpeed = cmd.getDigSpeed(); //How fast we want to dig
            targetDist = cmd.getDist();   //How far forward we want to go
            driveSpeed = cmd.getDriveSpeed(); //How fast do we want to go forward
            modeStartTime = Instant.now(); //Time stamp for when this message was received
            updateMotors(); //Start digging with given goals
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
         * Takes in message queue from RabbitMQ, interprets the message, and then starts drill end command.
         * handleDelivery method is called when the channel Object calls basicConsume method with DrillEndConsumer instance.
         *
         * @param consumerTag you can disregard this input. RabbitAMQP stuff
         * @param envelope    you can disregard this input. RabbitAMQP stuff
         * @param properties  will always be null for our purposes
         * @param body        the byte array representation of the incoming message itself
         * @throws IOException when message system messes up with us
         */
        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            currentJob = DrillJob.NONE; //Transition to DrillJob.NONE state to end digging cycle
            updateMotors(); //Ends digging cycle
        }
    }

	/*
	 * NOTE: These current and digging speed consumers are very janky right now because it uses PositionControlCommand to set up current limits. 
	 * This will be fixed shortly. In other modules, you should subscribe to statemodule and parse data from state update.
	 */

    /**
     * Consumer class for lower current message
     */
    private class LowerCurrentConsumer extends DefaultConsumer {
        public LowerCurrentConsumer(Channel channel) {
            super(channel);
        }

        /**
         * Takes in message queue from RabbitMQ, interprets the message, and sets up lower current limit.
         * handleDelivery method is called when the channel Object calls basicConsume method with LowerCurrentConsumer instance.
         *
         * @param consumerTag you can disregard this input. RabbitAMQP stuff
         * @param envelope    you can disregard this input. RabbitAMQP stuff
         * @param properties  will always be null for our purposes
         * @param body        the byte array representation of the incoming message itself
         * @throws IOException when message system messes up with us
         */
        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            Messages.PositionContolCommand cmd = Messages.PositionContolCommand.parseFrom(body);
            currentLowerLimit = cmd.getPosition();
            detectStall();
            updateMotors();
        }
    }

    /**
     * Consumer class for upper current message
     */
    private class UpperCurrentConsumer extends DefaultConsumer {
        public UpperCurrentConsumer(Channel channel) {
            super(channel);
        }

        /**
         * Takes in message queue from RabbitMQ, interprets the message, and then sets up upper current limit.
         * handleDelivery method is called when the channel Object calls basicConsume method with UpperCurrentConsumer instance.
         *
         * @param consumerTag you can disregard this input. RabbitAMQP stuff
         * @param envelope    you can disregard this input. RabbitAMQP stuff
         * @param properties  will always be null for our purposes
         * @param body        the byte array representation of the incoming message itself
         * @throws IOException when message system messes up with us
         */
        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            Messages.PositionContolCommand cmd = Messages.PositionContolCommand.parseFrom(body);
            currentUpperLimit = cmd.getPosition();
            detectStall();
            updateMotors();
        }
    }

    /**
     * Consumer class for dig speed.
     */
    private class DigSpeedConsumer extends DefaultConsumer {
        public DigSpeedConsumer(Channel channel) {
            super(channel);
        }

        /**
         * Takes in message queue from RabbitMQ, interprets the message, and then sets up digging speed.
         * handleDelivery method is called when the channel Object calls basicConsume method with DigSpeedConsumer instance.
         *
         * @param consumerTag you can disregard this input. RabbitAMQP stuff
         * @param envelope    you can disregard this input. RabbitAMQP stuff
         * @param properties  will always be null for our purposes
         * @param body        the byte array representation of the incoming message itself
         * @throws IOException when message system messes up with us
         */
        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            Messages.PositionContolCommand cmd = Messages.PositionContolCommand.parseFrom(body);
            digSpeed = cmd.getPosition();
            updateMotors();
        }
    }

    /**
     * enum to represent current task AutoDrillModule is performing.
     * When DrillJob is NONE, the module stops digging.
     */
    private enum DrillJob {
        DEEP, SURFACE, NONE
    }

    /*
     * RabbitMQ stuff
     */
    private String exchangeName;
    private Connection connection;
    private Channel channel;

    /**
     * Keeps track of currentJob the module is performing
     */
    private DrillJob currentJob = DrillJob.NONE;
    /**
     * Keeps track of lastJob the module is performing to resume once dealt with stall current
     */
    private DrillJob lastJob = DrillJob.NONE;
    /**
     * Depth we want to dig
     */
    private float targetDepth = 100.0F;
    /**
     * Distance we want to travel as we are digging
     */
    private float targetDist = 0.0F;
    /**
     * Digging speed that we want to dig in
     */
    private float digSpeed = 1.0F;
    /**
     * How fast we want to move forward as we dig
     */
    private float driveSpeed = 0.5F;
    /**
     * Records when digging command started. This gets updated every time getCurrentDepthTarget is called
     */
    private Instant modeStartTime;
    /**
     * Records the arm translation when digging command started This gets updated every time getCurrentDepthTarget is called
     * This value is basically subgoal of each motor command.
     */
    private float modeStartDepth = 10.0F;
    /**
     * Records whether the excavation motor is in stall
     */
    private boolean isStalled = false;

    /*Current status*/
    private float bc_trans = 0.0F;
    private float bc_angle = 0.0F;
    private float bc_current = 0.0F;

    /**
     * Depending on current task and current the excavation motor is pulling, this method sends out appropriate
     * motor control messages.
     */
    private void updateMotors() {

        try {
            switch (currentJob) {
                case NONE:
                    if (currentJob != lastJob) {
                        excavationConveyorRPM(0);
                        excavationTranslationControl(0);
                        locomotionStraight(0.0F);
                    }
                    break;
                case DEEP:
                    if (isStalled) {
                        excavationConveyorRPM(-100);
                        excavationTranslationControl(0);
                    } else {
                        excavationConveyorRPM(-100);
                        excavationTranslationControl(getCurrentDepthTarget());
                    }
                    break;
                case SURFACE:
                    if (isStalled) {
                        excavationConveyorRPM(-100);
                        excavationTranslationControl(0);
                    } else {
                        excavationConveyorRPM(-100);
                        excavationTranslationControl(getCurrentDepthTarget());
                        if (bc_trans < (targetDepth - 10)) {
                            locomotionStraight(0.0F);
                        } else {
                            locomotionStraight(driveSpeed);
                        }
                    }
                    break;
            }
            lastJob = currentJob;
        } catch (IOException e) {
            System.out.println("AutoDrill failed to publish message with exception:");
            e.printStackTrace();
        }
    }

    /**
     * This method compares current the motor is pulling and determine whether the motor is in stall
     */
    private void detectStall() {
        if (!isStalled && bc_current > currentUpperLimit) {
            // Transition to stalled
            isStalled = true;
        } else if (isStalled && bc_current <= currentLowerLimit) {
            // Transition to unstalled
            isStalled = false;
            modeStartTime = Instant.now();
            modeStartDepth = bc_trans;
        }
    }

    /**
     * This method calculates subgoal of our task so that our motor control commands are incremental.
     *
     * @return the calculated next target depth the next command should have.
     */
    private float getCurrentDepthTarget() {
        Instant now = Instant.now();
        float calculatedDepth = modeStartDepth + (Duration.between(modeStartTime, now).toMillis() / 1000.0F) * digSpeed;
        calculatedDepth = calculatedDepth < targetDepth ? calculatedDepth : targetDepth;
        modeStartDepth = calculatedDepth;
        modeStartTime = now;
        return calculatedDepth;
    }

    /**
     * Wrapper method for creating excavation translation command message
     *
     * @param targetValue the amount of translation we want
     * @throws IOException when failed to create/publish message
     */
    private void excavationTranslationControl(float targetValue) throws IOException {
        Messages.PositionContolCommand pcc = Messages.PositionContolCommand.newBuilder()
                .setPosition(targetValue)
                .setTimeout(123)
                .build();
        AutoDrillModule.this.channel.basicPublish(exchangeName, "motorcontrol.excavation.conveyor_translation_displacement", null, pcc.toByteArray());
    }

    /**
     * Wrapper method for creating excavation arm angle command message
     *
     * @param targetValue the angle of the arm we want
     * @throws IOException when failed to create/publish message
     */
    private void excavationAngleControl(float targetValue) throws IOException {
        Messages.PositionContolCommand pcc = Messages.PositionContolCommand.newBuilder()
                .setPosition(targetValue)
                .setTimeout(123)
                .build();
        AutoDrillModule.this.channel.basicPublish(exchangeName, "motorcontrol.excavation.arm_pos", null, pcc.toByteArray());
    }

    /**
     * Wrapper method for creating excavation conveyor RPM command message
     *
     * @param targetValue conveyor RPM that we want
     * @throws IOException when failed to create/publish message
     */
    private void excavationConveyorRPM(float targetValue) throws IOException {
        Messages.SpeedContolCommand msg = SpeedContolCommand.newBuilder()
                .setRpm(targetValue)
                .setTimeout(123)
                .build();
        AutoDrillModule.this.channel.basicPublish(exchangeName, "motorcontrol.excavation.bucket_conveyor_rpm", null, msg.toByteArray());
    }

	private void locomotionStraight(float speed) throws IOException{
		Messages.LocomotionControlCommandStraight msg = LocomotionControlCommandStraight.newBuilder()
				.setTimeout(123)
				.setSpeed(speed)
				.build();
		AutoDrillModule.this.channel.basicPublish(exchangeName, "subsyscommand.locomotion.straight", null, msg.toByteArray());
	}
	
	public AutoDrillModule(){
		this("amq.topic");
	}
	
	public AutoDrillModule(String exchangeName){
		this.exchangeName = exchangeName;
	}
	
	/**
	 * The method where everything gets set up to operate.
	 ** @throws IOException
	 * @throws TimeoutException
	 */
	public void runWithExceptions() throws IOException, TimeoutException{
		//Setup connection
		ConnectionFactory factory = new ConnectionFactory();
		factory.setHost("localhost");
		this.connection = factory.newConnection();
		this.channel = connection.createChannel();
		
		//Listen for DrillDeep command
		String queueName = channel.queueDeclare().getQueue();
		channel.queueBind(queueName, exchangeName, "subsyscommand.excavation.dig_deep");
		this.channel.basicConsume(queueName, true, new DrillDeepConsumer(channel));
		
		//Listen for DrillSurface command
		queueName = channel.queueDeclare().getQueue();
		channel.queueBind(queueName, exchangeName, "subsyscommand.excavation.dig_surface");
		this.channel.basicConsume(queueName, true, new DrillSurfaceConsumer(channel));
		
		//Listen for DrillEnd command
		queueName = channel.queueDeclare().getQueue();
		channel.queueBind(queueName, exchangeName, "subsyscommand.excavation.dig_end");
		this.channel.basicConsume(queueName, true, new DrillEndConsumer(channel));

        //Listen for lower current
        queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "subsyscommand.excavation.lower_current");
        this.channel.basicConsume(queueName, true, new LowerCurrentConsumer(channel));

        //Listen for upper current
        queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "subsyscommand.excavation.upper_current");
        this.channel.basicConsume(queueName, true, new UpperCurrentConsumer(channel));

        //Listen for dig speed
        queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "subsyscommand.excavation.dig_speed");
        this.channel.basicConsume(queueName, true, new DigSpeedConsumer(channel));

        //Subscribing to StateModule
        Messages.StateSubscribe msg = Messages.StateSubscribe.newBuilder()
                .setReplyKey("autoDrillModule")
                .setInterval(0.1F)
                .setDepositionDetailed(false)
                .setDepositionSummary(true)
                .setExcavationDetailed(true)
                .setExcavationSummary(false)
                .setLocomotionDetailed(true)
                .setLocomotionSummary(false)
                .build();
        this.channel.basicPublish(exchangeName, "state.subscribe", null, msg.toByteArray());

        // Enter main loop
        modeStartTime = Instant.now();
        queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "autoDrillModule");
        this.channel.basicConsume(queueName, true, new DefaultConsumer(channel) {
            @Override
            public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
                Messages.State msg = Messages.State.parseFrom(body);
                bc_trans = msg.getExcDetailed().getDisplacement();
                bc_current = msg.getExcDetailed().getConveyorMotorCurrent();
                bc_angle = msg.getExcDetailed().getArmPos();

			    detectStall();
				updateMotors();
			}
		});
		
	}
	
	public  static void main(String[] args){
		AutoDrillModule module = new AutoDrillModule();
		module.start();
	}
}
