package com.cwrubotix.glennifer.automodule;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeoutException;

import javafx.application.Application;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.stage.Stage;
import javafx.scene.Scene;
import javafx.scene.layout.HBox;
import javafx.scene.control.Button;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.LocObsStateDetailed;
import com.cwrubotix.glennifer.Messages.LocalizationPosition;
import com.cwrubotix.glennifer.Messages.ObstaclePosition;
import com.rabbitmq.client.AMQP;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

/**
 * Test Module to test ArcPath separate from AutoModule/Transit.
 * @author Seohyun Jung
 *
 */

public class ArcPathTestModule extends Module{
    public enum Direction {FORWARD, BACKWARD};
    private Direction direction;
    private Position destination;
    private Position currentPos;
    private Position[] scheme = {new Position(1.0, 6.0), new Position(-1.0, 1.0), new Position(0.0, 6.0), new Position(1.0, 1.0), 
	    			new Position(-1.0, 6.0), new Position(0.0, 1.0), new Position(1.0,6.0)};
    private boolean tagFound = false;
    private boolean launched = false;
    private double constant = 1.0;
    private int progress = 1;
    private int currentScheme = 0;
    private ArcPath arcPath;
    private final float DRIVE_SPEED = 20;
    private final double rate = 0.015; //Random shit
    
    public ArcPathTestModule(){
	this.exchangeName = "amq.topic";
    }
    
    private void setUpTest() throws IOException{
	destination = scheme[currentScheme];
	if(currentPos.getY() < destination.getY()){
	    direction = Direction.FORWARD;
	    arcPath = new ArcPath(currentPos, destination);
	}else{
	    direction = Direction.BACKWARD;
	    arcPath = new ArcPath(destination, currentPos);
	}
    }
    
    private void setUpTest(int scheme){
	destination = this.scheme[scheme];
	if(currentPos.getY() < destination.getY()){
	    direction = Direction.FORWARD;
	    arcPath = new ArcPath(currentPos, destination);
	}else{
	    direction = Direction.BACKWARD;
	    arcPath = new ArcPath(destination, currentPos);
	}
	progress = 1;
	moveRobot();
    }
    
    private boolean checkStray(){
	double[] eqn = arcPath.getArc(progress);
	double y = eqn[0] * currentPos.getX() * currentPos.getX() * currentPos.getX() +
		eqn[1] * currentPos.getX() * currentPos.getX() +
		eqn[2] * currentPos.getX() +
		eqn[3];
	if(Math.abs(currentPos.getY() - y) > 0.3){
	    arcPath.newPath(currentPos, destination);
	    progress = 1;
	    return true;
	} return false;
    }
    
    private void updateConstant(){
	double[] args = arcPath.getArc(progress);
	Curvature k = new Curvature(arcPath.getPoints()[progress - 1].getX(), arcPath.getPoints()[progress].getX(), arcPath.getArc(progress));
	double expectedY = args[0] * currentPos.getX() * currentPos.getX() * currentPos.getX() +
			   args[1] * currentPos.getX() * currentPos.getX() +
			   args[2] * currentPos.getX() +
			   args[3];
	
	if(Math.abs(currentPos.getY() - expectedY) > 0.3F){
	    if(currentPos.getY() - expectedY > 0){
		constant = constant + (currentPos.getY() - expectedY) * rate;
	    }
	} else{
	    double m = 0.0;
	    if(currentPos.getHeading() == Math.PI / 2 || currentPos.getHeading() == Math.PI * 3 / 2)
		m = 0;
	    else
		m = -Math.tan(currentPos.getHeading() + Math.PI / 2);
	    double expectedM = k.getFirstDeriv(currentPos.getX());
	    if(expectedM - m > 0){
		switch(k.getTurn(currentPos.getX())){
		case LEFT: constant = constant + (currentPos.getY() - expectedY) * rate;
		    break;
		case RIGHT: constant = constant - (currentPos.getY() - expectedY) * rate;
		    break;
		}
	    }else{
		switch(k.getTurn(currentPos.getX())){
		case LEFT: constant = constant - (currentPos.getY() - expectedY) * rate;
		    break;
		case RIGHT: constant = constant + (currentPos.getY() - expectedY) * rate;
		    break;
		}
	    }
	}
	
    }
    
    private boolean checkProgress(){
	if(!checkStray()){
	    Position p1 = arcPath.getPoints()[progress - 1];
	    Position p2 = arcPath.getPoints()[progress];
	    if(currentPos.getY() < Math.min(p1.getY(), p2.getY()) || currentPos.getY() > Math.max(p1.getY(), p2.getY())
		    || currentPos.getX() < Math.min(p1.getX(), p2.getX()) || currentPos.getX() > Math.max(p1.getX(), p2.getX())){
		if((progress != arcPath.getPoints().length - 1) || currentPos.equals(arcPath.getPoints()[progress])){
		    progress++;
		    return false;
		} else{
		    return true;
		}
	    }
	}return false;
    }
    
    private void moveRobot(){
	if(launched){
	    System.out.println("Moving robot");
	    Position p1 = null,p2 = null;
	    Curvature k = null;
	    switch(direction){
	    case FORWARD: p1 = arcPath.getPoints()[progress - 1]; p2 = arcPath.getPoints()[progress];
	    	k = new Curvature(p1.getX(), p2.getX(), arcPath.getArc(progress));
		break;
	    case BACKWARD:
		int place = arcPath.getPoints().length - progress;
		p1 = arcPath.getPoints()[place]; p2 = arcPath.getPoints()[place - 1];
		k = new Curvature(p1.getX(), p2.getX(), arcPath.getArc(place));
		break;
	    }
	    double factor = DRIVE_SPEED / (1 + Math.exp(constant / k.getCurvature(currentPos.getX())));
	    float left = 0.0F, right = 0.0F;
	    switch(k.getTurn(currentPos.getX())){
	    case LEFT: System.out.println("Turing left...");
		left = (float)Math.min(factor, factor * Math.exp(constant / k.getCurvature(currentPos.getX())));
		right = (float)Math.max(factor, factor * Math.exp(constant / k.getCurvature(currentPos.getX())));
		break;
	    case RIGHT: System.out.println("Turning right...");
		left = (float)Math.max(factor, factor * Math.exp(constant / k.getCurvature(currentPos.getX())));
		right = (float)Math.min(factor, factor * Math.exp(constant / k.getCurvature(currentPos.getX())));
		break;
	    }
	    
	    switch(direction){
	    case FORWARD: System.out.println("Going forward direction"); leftMotorControl(left); rightMotorControl(right);
		break;
	    case BACKWARD:System.out.println("Going backward direction"); leftMotorControl(-left); rightMotorControl(-right);
		break;
	    }
	}
    }
    
    public void endTest(){
	leftMotorControl(0.0F);
	rightMotorControl(0.0F);
	this.stop();
	System.exit(0);
    }
    
    private void leftMotorControl(float value){
	Messages.SpeedControlCommand msg = Messages.SpeedControlCommand.newBuilder()
						   .setRpm(value)
						   .setTimeout(0)
						   .build();
	try{
	    this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.front_left.wheel_rpm", null, msg.toByteArray());
	    this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.back_left.wheel_rpm", null, msg.toByteArray());
	} catch(IOException e){
	    e.printStackTrace();
	}
    }
    
    private void rightMotorControl(float value){
	Messages.SpeedControlCommand msg = Messages.SpeedControlCommand.newBuilder()
		   .setRpm(value)
		   .setTimeout(0)
		   .build();
	try{
	    this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.front_right.wheel_rpm", null, msg.toByteArray());
	    this.channel.basicPublish(exchangeName, "motorcontrol.locomotion.back_right.wheel_rpm", null, msg.toByteArray());
	} catch(IOException e){
	    e.printStackTrace();
	}
    }

    @Override
    protected void runWithExceptions() throws IOException, TimeoutException {
	ConnectionFactory factory = new ConnectionFactory();
	factory.setHost("localhost");
	this.connection = factory.newConnection();
	this.channel = connection.createChannel();
	
	// Subscribing to StateModule
	Messages.StateSubscribe msg = Messages.StateSubscribe.newBuilder().setReplyKey("arcPathTest")
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
	channel.queueBind(queueName, exchangeName, "arcPathTest");
	this.channel.basicConsume(queueName, true, new StateUpdate(channel));
	
	System.out.println("Waiting on Localization message");
    }
    
    public static void main(String[] args){
	Control.launchwrap(args);
    }
    
    public static class Control extends Application{
	private ArcPathTestModule module;
	
	@Override
	public void start(Stage primaryStage){
	    module = new ArcPathTestModule();
	    module.start();
	    HBox box = new HBox();
	    Button start = new Button("START");
	    Button estop = new Button("ESTOP");
	    start.setOnAction(new EventHandler<ActionEvent>(){
		public void handle(ActionEvent e){
		    if(module.tagFound){
			try{
			    module.setUpTest();
			}catch(IOException er){
			    er.printStackTrace();
			}
			module.launched = true;
		    }
		}
	    });
	    estop.setOnAction(new EventHandler<ActionEvent>(){
		public void handle(ActionEvent e){
		    module.endTest();
		}
	    });
	    box.getChildren().addAll(start, estop);
	    Scene scene = new Scene(box);
	    primaryStage.setScene(scene);
	    primaryStage.sizeToScene();
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
	    
	    for(ObstaclePosition op : obstacles){
		Obstacle obs = new Obstacle(new Position(op.getXPosition(), op.getYPosition()));
		if(arcPath.addObstacle(currentPos, obs)){
		    progress = 1;
		}
	    }
	    if(progress != 1){
		updateConstant();
		checkProgress();
		if(progress == arcPath.getPoints().length - 1)
		    setUpTest(++currentScheme);
	    }
	    moveRobot();
	}
    }
}
