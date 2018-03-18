package test;

import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;
import com.cwrubotix.glennifer.Messages.UnixTime;

import java.io.IOException;
import java.time.Instant;
import java.util.concurrent.TimeoutException;

import main.java.com.cwrubotix.glennifer.automodule.Module;

/**
 * A test module to confirm AutoModule is keeping time correctly.
 * <p>
 * This module will keep track of launch/stop messages that are coming from AutoModule and send appropriate messages with
 * reasonable time intervals to make sure AutoModule is properly working.
 * </p>
 * 
 * TODO:
 *    1) Decide on how much time we should give before softStop and add feature to test it.
 *    2) Add feature to test whether inputs in messages are valid.
 *    3) Add feature to test whether the run is within the time limit.
 *    4) Add checker to end the module after normal operation.
 *    
 * @author Seohyun Jung
 *
 */
public class AutoModuleTester extends Module{
    
    /***
     * Represents the current state of the mining cycle.
     * <ul>
     * <li>IDLE : initial state before the mining run starts.</li>
     * <li>TRANSIT_A : Transit state from dumping area to digging area</li>
     * <li>TRANSIT_B : Transit state from digging area to dumping area</li>
     * <li>DRILL : Drill state in digging area</li>
     * <li>DUMP : Dump state in dumping area</li>
     * </ul>
     * @author Seohyun Jung
     *
     */
    private enum State {IDLE, TRANSIT_A, TRANSIT_B, DRILL, DUMP};
    /**Stores the current state of the test*/
    private State currentState = State.IDLE;
    /**Indicates whether the first run is over*/
    private boolean firstRun = true;
    /**Indicates whether the robot is performing tasks*/
    private boolean taskAssigned = false;
    /**Constant storing time out threshold which is set to 5 millisecond in unit of nanoseconds*/
    private final long TIMEOUT = 5000000L;
    /**Constant storing the error bound allowed between message instants. Set to half a millisecond in unit of nanoseconds*/
    private final long TIME_ERROR_BOUND = 500000L;
    /**Keeps track of when the mining run started*/
    private Instant startTimeStamp;
    /**Keeps track of the time the last message was received from AutoModule*/
    private Instant lastTimeStamp;
    /**Keeps track of the time the last softStop message was received if any after last hardStop message*/
    private Instant softStopTimeStamp;
    /**Keeps track of time left given from last softStop*/
    private float timeLeft;
    
    public AutoModuleTester(String exchangeName){
	this.exchangeName = exchangeName;
    }
    
    public AutoModuleTester(){
	this("amq.topic");
    }
    
    
    @Override
    public void runWithExceptions() throws IOException, TimeoutException {
	//Setup connection
	ConnectionFactory factory = new ConnectionFactory();
	factory.setHost("localhost");
	this.connection = factory.newConnection();
	this.channel = connection.createChannel();
	
	//Listen for launchTransit command
	String queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "launch.transit");
	this.channel.basicConsume(queueName, true, new LaunchTransitListener(channel));
	
	//Listen for launchDrill command
	queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "launch.drill");
	this.channel.basicConsume(queueName, true, new LaunchDrillListener(channel));
	
	//Listen for launchDump command
	queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "launch.dump");
	this.channel.basicConsume(queueName, true, new LaunchDumpListener(channel));
	
	/*TODO Check whether one consumer can be bound to multiple messages.*/
	
	//Listen for SoftStop command
	queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "softstop.transit");
	channel.queueBind(queueName, exchangeName, "softstop.drill");
	channel.queueBind(queueName, exchangeName, "softstop.dump");
	this.channel.basicConsume(queueName, true, new SoftStopListener(channel));
	
	//Listen for HardStop command
	queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "hardstop.transit");
	channel.queueBind(queueName, exchangeName, "hardstop.drill");
	channel.queueBind(queueName, exchangeName, "hardstop.dump");
	this.channel.basicConsume(queueName, true, new HardStopListener(channel));
    }
    
    public static void main(String[] args){
	AutoModuleTester module = new AutoModuleTester();
	module.start();
    }
    
    private void switchState(State nextState){
	switch(currentState){
	case IDLE:
	    startTimeStamp = Instant.now();
	    if(!nextState.equals(State.TRANSIT_A)){
		if(firstRun){
		    System.err.println("ERROR\n Current State: MINING CYCLE I: IDLE");
		} else{
		    System.err.println("ERROR\n SHOULD NOT BE IN IDLE AFTER FIRST RUN");
		    System.exit(1);
		}
		System.err.println("INVALID MESSAGE: Tried to switch from IDLE state to invalid state");
		System.exit(1);
	    }
	    break;
	case TRANSIT_A:
	    if(!nextState.equals(State.DRILL)){
		if(firstRun){
		    System.err.println("ERROR\n Current State: MINING CYCLE I: TRANSIT_A");
		} else{
		    System.err.println("ERROR\n Current State: MINING CYCLE II: TRANSIT_A");
		}
		System.err.println("INVALID MESSAGE: Tried to switch from Transit_A state to invalid state");
		System.exit(1);
	    }
	    break;
	case TRANSIT_B:
	    if(!nextState.equals(State.DUMP)){
		if(firstRun){
		    System.err.println("ERROR\n Current State: MINING CYCLE I: TRANSIT_B");
		} else{
		    System.err.println("ERROR\n Current State: MINING CYCLE II: TRANSIT_B");
		}
		System.err.println("INVALID MESSAGE: Tried to switch from TRANSIT_B state to invalid state");
		System.exit(1);
	    }
	    break;
	case DRILL:
	    if(!nextState.equals(State.TRANSIT_B)){
		if(firstRun){
		    System.err.println("ERROR\n Current State: MINING CYCLE I: DRILL");
		} else{
		    System.err.println("ERROR\n Current State: MINING CYCLE II: DRILL");
		}
		System.err.println("INVALID MESSAGE: Tried to switch from Drill state to invalid state");
		System.exit(1);
	    }
	    break;
	case DUMP:
	    if(!nextState.equals(State.TRANSIT_A)){
		if(firstRun){
		    System.err.println("ERROR\nCurrent State: MINING CYCLE I: DUMP");
		} else{
		    System.err.println("ERROR\nCurrent State: MINING CYCLE II: DUMP");
		}
		System.err.println("INVALID MESSAGE: Tried to switch from Dump state to invalid state");
		System.exit(1);
	    }
	    break;
	default:
	    System.err.println("ERROR: Encountered empty current state.");
	    System.exit(1);
	}
	currentState = nextState;
	taskAssigned = true;
    }
    
    private void checkTimeOut(Instant time){
	if(time.getNano() - lastTimeStamp.getNano() >= TIMEOUT){
	    System.err.println("ERROR: Took too much time for Automodule to configure next job.");
	    System.exit(1);
	} else{
	    lastTimeStamp = time;
	}
    }
    
    private void checkValidLaunch(){
	if(taskAssigned){
	    System.err.println("ERROR: Tried to launch other task while the task is on progress");
	    System.exit(1);
	}
    }
    
    private void checkValidStop(){
	if(!taskAssigned){
	    System.err.println("ERROR: Tried to stop a non-existing task");
	    System.exit(1);
	}
    }
    
    private void checkTimeSinceSoftStop(Instant time){
	if(softStopTimeStamp == null){
	    System.err.println("ERROR: Received hardStop message without previous softStop message");
	    System.exit(1);
	}
	UnixTime softStop = instantToUnixTime(softStopTimeStamp);
	UnixTime hardStop = instantToUnixTime(time);
	if(Math.abs(timeLeft - (hardStop.getTimeFrac() - softStop.getTimeFrac())) <= TIME_ERROR_BOUND){
	    softStopTimeStamp = null;
	    taskAssigned = false;
	} else{
	    System.err.println("ERROR: hardStop message was not sent in timely manner.");
	    System.exit(1);
	}
    }

    private class LaunchTransitListener extends DefaultConsumer{

	public LaunchTransitListener(Channel channel) {
	    super(channel);
	}
	
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException{
	    checkValidLaunch();
	    checkTimeOut(Instant.now());
	    if(currentState.equals(State.IDLE) || currentState.equals(State.DUMP)){
		switchState(State.TRANSIT_A);
	    } else{
		switchState(State.TRANSIT_B);
	    }
	}
	
    }
    
    private class LaunchDrillListener extends DefaultConsumer{

	public LaunchDrillListener(Channel channel) {
	    super(channel);
	}
	
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException{
	    checkValidLaunch();
	    checkTimeOut(Instant.now());
	    switchState(State.DRILL);
	}
	
    }
    
    private class LaunchDumpListener extends DefaultConsumer{

	public LaunchDumpListener(Channel channel) {
	    super(channel);
	}
	
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException{
	    checkValidLaunch();
	    checkTimeOut(Instant.now());
	    switchState(State.DUMP);
	}
	
    }
    
    private class SoftStopListener extends DefaultConsumer{
	
	public SoftStopListener(Channel channel){
	    super(channel);
	}
	
	@Override
	public void handleDelivery(String conumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException{
	    checkValidStop();
	    softStopTimeStamp = Instant.now();
	}
    }
    
    private class HardStopListener extends DefaultConsumer{

	public HardStopListener(Channel channel) {
	    super(channel);
	}
	
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException{
	    checkTimeSinceSoftStop(Instant.now());
	}
	
    }
    
}