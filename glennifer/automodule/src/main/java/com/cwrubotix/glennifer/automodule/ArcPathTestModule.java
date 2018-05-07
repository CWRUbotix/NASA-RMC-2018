package com.cwrubotix.glennifer.automodule;

import java.io.IOException;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.util.LinkedList;
import java.util.concurrent.TimeoutException;

import javafx.application.Application;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.stage.Stage;
import javafx.scene.Scene;
import javafx.scene.layout.HBox;
import javafx.scene.control.Button;

import com.cwrubotix.glennifer.Messages;
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
    private Position[] scheme;
    private boolean tagFound = false;
    private boolean launched = false;
    private double constant = 1.0;
    private int progress = 1;
    private int currentScheme = 0;
    private ArcPath arcPath;
    private final float DRIVE_SPEED = 15;
    private final double rate = 0.015; //Random shit
    
    public ArcPathTestModule(){
	this.exchangeName = "amq.topic";
    }
    
    private void setUpTest() throws IOException{
	loadScheme();
	//loadConstant();
	destination = scheme[currentScheme];
	if(currentPos.getY() < destination.getY()){
	    direction = Direction.FORWARD;
	    arcPath = new ArcPath(currentPos, destination);
	}else{
	    direction = Direction.BACKWARD;
	    arcPath = new ArcPath(destination, currentPos);
	}
	moveRobot();
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
    
    private void loadScheme() throws IOException{
	File scheme = new File("test/testScheme.txt");
	BufferedReader reader = new BufferedReader(new FileReader(scheme));
	LinkedList<Position> positions = new LinkedList<>();
	String line = reader.readLine();
	while(line != null){
	    String[] args = line.split(" ");
	    positions.add(new Position(Double.parseDouble(args[0]), Double.parseDouble(args[1])));
	    line = reader.readLine();
	}
	reader.close();
	this.scheme = positions.toArray(new Position[0]);
    }
    
    private void loadConstant() throws IOException{
	File constant = new File("test/constant.txt");
	if(constant.exists()){
	    BufferedReader reader = new BufferedReader(new FileReader(constant));
	    String temp = reader.readLine();
	    String line = null;
	    while(temp != null){
		line = temp;
		temp = reader.readLine();
	    }
	    this.constant = Double.parseDouble(line.split(":")[1].trim());
	    reader.close();
	} else{
	    this.constant = 1.0;
	}
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
	double expectedY = args[0] * currentPos.getX() * currentPos.getX() * currentPos.getX() +
			   args[1] * currentPos.getX() * currentPos.getX() +
			   args[2] * currentPos.getX() +
			   args[3];
	constant = constant + (currentPos.getY() - expectedY) * rate;
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
	    Curvature k = new Curvature(arcPath.getPoints()[progress - 1].getX(), arcPath.getPoints()[progress].getX(), arcPath.getArc(progress));
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
    
    
    private void logConstant(){
	try{
	    File constant = new File("test/constant.txt");
	    if(!constant.exists())
		constant.createNewFile();
	    FileWriter fw = new FileWriter(constant);
	    fw.append(constant + "\n");
	    fw.close();
	} catch(IOException e){
	    e.printStackTrace();
	}
    }
    
    public void endTest(){
	logConstant();
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
	
	String queueName = channel.queueDeclare().getQueue();
	this.channel.queueBind(queueName, exchangeName, "loc.post");
	this.channel.basicConsume(queueName, true, new LocConsumer(channel));
	
	queueName = channel.queueDeclare().getQueue();
	this.channel.queueBind(queueName, exchangeName, "obstacle.position");
	this.channel.basicConsume(queueName, true, new ObsConsumer(channel));
	
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
    
    private class LocConsumer extends DefaultConsumer{
	public LocConsumer(Channel channel){
	    super(channel);
	}
	
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException{
	    Messages.LocalizationPosition msg = Messages.LocalizationPosition.parseFrom(body);
	    currentPos = new Position(msg.getXPosition(), msg.getYPosition(), msg.getBearingAngle());
	    if(!tagFound){
		tagFound = true;
		System.out.println("Found AprilTag, Ready to launch");
	    }
	    if(launched){
		if(checkProgress()){
		    System.out.println("Starting next iteration");
		    setUpTest(++currentScheme);
		}
		updateConstant();
		moveRobot();
	    }
	}
    }
    
    private class ObsConsumer extends DefaultConsumer{
	public ObsConsumer(Channel channel){
	    super(channel);
	}
	
	@Override
	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException{
	    Messages.ObstaclePosition msg = Messages.ObstaclePosition.parseFrom(body);
	    Obstacle obs = new Obstacle(msg.getXPosition(), msg.getYPosition(), msg.getDiameter() / 2);
	    if(launched && arcPath.addObstacle(currentPos, obs)){
		progress = 1;
		moveRobot();
	    }
	}
    }
}
