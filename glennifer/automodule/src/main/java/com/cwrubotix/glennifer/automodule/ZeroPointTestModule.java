package com.cwrubotix.glennifer.automodule;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeoutException;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.LocObsStateDetailed;
import com.cwrubotix.glennifer.Messages.LocalizationPosition;
import com.cwrubotix.glennifer.Messages.ObstaclePosition;
import com.cwrubotix.glennifer.automodule.PathFinder.DestinationModified;
import com.cwrubotix.glennifer.automodule.PathFindingAlgorithm.AlgorithmFailureException;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.layout.HBox;
import javafx.stage.Stage;

public class ZeroPointTestModule extends Module{
    
    private boolean tagFound = false;
    private boolean launched = false;
    private Position currentPos;
    private Position destination;
    private PathFinder finder;
    private final float DRIVE_SPEED = 30F;
    private final float TURNING_SPEED = 30F;
    public enum Direction {FORWARD, BACKWARD};
    private Direction direction;
    private Position[] scheme = {new Position(1.0, 6.0), new Position(-1.0, 1.0), new Position(0.0, 6.0), new Position(1.0, 1.0), 
		new Position(-1.0, 6.0), new Position(0.0, 1.0), new Position(1.0,6.0)};
    private int currentScheme = 0;
    private int progress = 1;
    private int lastProgress = 0;
    
    public ZeroPointTestModule(){
	this.exchangeName = "amq.topic";
    }
    
    private void setUpTest(){
	destination = scheme[currentScheme];
	if(currentPos.getY() < destination.getY()){
	    direction = Direction.FORWARD;
	}else{
	    direction = Direction.BACKWARD;
	}
	finder = new PathFinder(currentPos, destination);
    }
    
    private void setUpTest(int scheme){
	destination = this.scheme[scheme];
	if(currentPos.getY() < destination.getY()){
	    direction = Direction.FORWARD;
	    finder = new PathFinder(currentPos, destination);
	}else{
	    direction = Direction.BACKWARD;
	    finder = new PathFinder(destination, currentPos);
	}
	progress = 1;
    }
    
    private void checkStray(){
	if(Math.abs(currentPos.getHeading() - finder.getPath().getPoint(progress - 1).getHeadingTo(finder.getPath().getPoint(progress))) > 0.07){
	    finder.recalculatePath(currentPos, destination);
	    progress = 1;
	}
    }
    
    private void checkProgress(){
	checkStray();
	if(currentPos.equals(finder.getPath().getPoint(progress)))
	    progress++;
	if(currentPos.equals(destination)){
	    setUpTest(++currentScheme);
	    System.out.println("Reached current destination: " + finder.getPath().getPoint(progress - 1) 
		    + "\nNow heading to" + finder.getPath().getPoint(progress));
	    motorControl(0);
	}
    }
    
    private void moveRobot(){
	if(launched){
	    double heading = currentPos.getHeadingTo(finder.getPath().getPoint(progress));
	    if(direction == Direction.BACKWARD)
		heading = (heading + Math.PI) % (Math.PI * 2);
	    if(Math.abs(heading - currentPos.getHeading()) > 0.02){
		turn(heading);
		if(lastProgress != progress){
		    Messages.AutonomyNextHeading anh = Messages.AutonomyNextHeading.newBuilder().setHeading((float)heading).build();
		    try {
			this.channel.basicPublish(exchangeName, "autonomy.next_heading", null, anh.toByteArray());
		    } catch (IOException e) {
			System.err.println("Failed to send next heading information.");
			e.printStackTrace();
		    }
		}
	    }
	    else{
	    	if(direction == Direction.FORWARD)
				motorControl(DRIVE_SPEED);
			else
				motorControl(-DRIVE_SPEED);
	    }
	    lastProgress = progress;
	}
    }
    
    private void turn(double angle){
	if(Math.abs(angle - currentPos.getHeading()) > Math.PI){ //turn left
	    motorControl(TURNING_SPEED * -1, TURNING_SPEED, TURNING_SPEED * -1, TURNING_SPEED);
	}else{ //turn right
	    motorControl(TURNING_SPEED, TURNING_SPEED * -1, TURNING_SPEED, TURNING_SPEED * -1);
	}
    }
    
    public void endTest(){
	motorControl(0);
	System.exit(0);
    }
    
    private void motorControl(float fl, float fr, float bl, float br){
	try {
	    Messages.SpeedControlCommand msg = Messages.SpeedControlCommand.newBuilder()
		    .setRpm(fl)
		    .setTimeout(0)
		    .build();
	    this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.front_left.wheel_rpm", null, msg.toByteArray());
	    msg = Messages.SpeedControlCommand.newBuilder()
		    .setRpm(fr)
		    .setTimeout(0)
		    .build();
	    this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.front_right.wheel_rpm", null, msg.toByteArray());
	    msg = Messages.SpeedControlCommand.newBuilder()
		    .setRpm(bl)
		    .setTimeout(0)
		    .build();
	    this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.back_left.wheel_rpm", null, msg.toByteArray());
	    msg = Messages.SpeedControlCommand.newBuilder()
		    .setRpm(br)
		    .setTimeout(0)
		    .build();
	    this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.back_right.wheel_rpm", null, msg.toByteArray());
	} catch (IOException e) {
	    e.printStackTrace();
	}
    }
    
    private void motorControl(float value){
	motorControl(value, value, value, value);
    }
    
    @Override
    protected void runWithExceptions() throws IOException, TimeoutException {
	ConnectionFactory factory = new ConnectionFactory();
	factory.setHost("localhost");
	this.connection = factory.newConnection();
	this.channel = connection.createChannel();
	
	// Subscribing to StateModule
	Messages.StateSubscribe msg = Messages.StateSubscribe.newBuilder().setReplyKey("zeroPointTest")
									  .setInterval(0.1F)
									  .setDepositionDetailed(false)
									  .setDepositionSummary(true)
									  .setExcavationDetailed(false)
									  .setExcavationSummary(true)
									  .setLocomotionDetailed(true)
									  .setLocomotionSummary(false)
									  .setLocObsDetailed(true)
									  .build();
	this.channel.basicPublish(exchangeName, "state.subscribe", null, msg.toByteArray());
	
	String queueName = channel.queueDeclare().getQueue();
	channel.queueBind(queueName, exchangeName, "zeroPointTest");
	this.channel.basicConsume(queueName, true, new StateUpdate(channel));
	
	System.out.println("Waiting on Localization message");
    }
    
    @Override
    public void stop(){
	//Unsubscribing
	Messages.StateSubscribe msg = Messages.StateSubscribe.newBuilder().setReplyKey("zeroPointTest")
		  .setInterval(0.1F)
		  .setDepositionDetailed(false)
		  .setDepositionSummary(true)
		  .setExcavationDetailed(false)
		  .setExcavationSummary(true)
		  .setLocomotionDetailed(true)
		  .setLocomotionSummary(false)
		  .setLocObsDetailed(true)
		  .build();
	try {
	    this.channel.basicPublish(exchangeName, "state.unsubscribe", null, msg.toByteArray());
	} catch (IOException e) {
	    e.printStackTrace();
	} finally{
	    super.stop();
	}
    }
    
    public static void main(String[] args){
	Control.launchwrap(args);
    }
    
    public static class Control extends Application{
	
	@Override
	public void start(Stage primaryStage){
	    ZeroPointTestModule module = new ZeroPointTestModule();
	    module.start();
	    HBox box = new HBox();
	    Button start = new Button("START");
	    Button estop = new Button("END");
	    start.setOnAction(e -> {
		if(module.tagFound){
		    module.setUpTest();
		    module.launched = true;
		}
	    });
	    estop.setOnAction(e -> module.endTest());
	    box.getChildren().addAll(start, estop);
	    Scene scene = new Scene(box);
	    primaryStage.setScene(scene);
	    primaryStage.sizeToScene();
	    primaryStage.setOnCloseRequest(e -> e.consume());
	    primaryStage.show();
	}
	
	private static void launchwrap(String[] args){
	    Application.launch(args);
	}
    }
    
    private class StateUpdate extends DefaultConsumer{
	public StateUpdate(Channel channel){
	    super(channel);
	}
	
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException{
	    Messages.State msg = Messages.State.parseFrom(body);
	    LocObsStateDetailed locObs = msg.getLocObsDetailed();
	    LocalizationPosition posUpdate = locObs.getLocPosition();
	    List<ObstaclePosition> obstacles = locObs.getObstaclesList();
	    Position pos = new Position(posUpdate.getXPosition(), posUpdate.getYPosition(), posUpdate.getBearingAngle());
	    if(!pos.equals(new Position(0, 0, 0))){
		tagFound = true;
		currentPos = pos;
	    }
	    if(tagFound)
		    checkProgress();
	   
	    
	    for(ObstaclePosition op : obstacles){
		Obstacle obs = new Obstacle(new Position(op.getXPosition(), op.getYPosition()));
		finder.setCurrentPos(currentPos);
		try {
		    finder.registerObstacle(obs);
		    progress = 1;
		} catch (DestinationModified e) {
		    destination = new Position(e.getX(), e.getY());
		} catch (AlgorithmFailureException e1){
		    System.err.println("Failed to create path");
		    endTest();
		}
		System.out.println("Obstacle added");
	    }

	    moveRobot();
	}
    }
}