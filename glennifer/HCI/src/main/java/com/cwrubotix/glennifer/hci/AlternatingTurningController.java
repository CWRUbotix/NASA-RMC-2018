package com.cwrubotix.glennifer.hci;

import com.cwrubotix.glennifer.Messages;
import com.rabbitmq.client.*;
import com.google.protobuf.InvalidProtocolBufferException;
import java.io.IOException;
import java.util.concurrent.TimeoutException;

public class AlternatingTurningController implements Runnable {

	private static final String motorTopic = "motorcontrol.#";
	private static String exchangeName = "amq.topic";
    private static Channel channel;
    private static String queueName;
	private static HardwareControlInterface hci;
	private static double rpm;
	private static boolean dir; //true is right, false is left
	private static boolean diagonal = true; //true is FL-BR, false is FR-BL
	private static boolean canContinue = true;

	public void runWithExceptions() throws IOException, TimeoutException{

		ConnectionFactory factory = new ConnectionFactory();
		factory.setHost("localhost");
		Connection connection = factory.newConnection();
		channel = connection.createChannel();
		queueName = channel.queueDeclare().getQueue();
		channel.queueBind(queueName, exchangeName, motorTopic);

		Consumer consumer = new DefaultConsumer(channel) {
			@Override
			public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body){
				String routingKey = envelope.getRoutingKey();
				String[] keys = routingKey.split("\\.");
				if (keys[0] == "motorcontrol"){
					canContinue = false;
				}
		    };
		};

		channel.basicConsume(queueName, true, consumer);

		try{
			while(canContinue){
				if(dir){
					if(diagonal){
						queueActuation(0, rpm);
	        			queueActuation(1, 0);
	        			queueActuation(2, 0);
	        			queueActuation(3, -rpm);
	        			Thread.sleep(2000);
	        			diagonal = false;
					}
					else{
						queueActuation(0, 0);
	        			queueActuation(1, -rpm);
	        			queueActuation(2, rpm);
	        			queueActuation(3, 0);
	        			Thread.sleep(2000);
	        			diagonal = true;
					}
				}
				else{
					if(!diagonal){
						queueActuation(0, 0);
	        			queueActuation(1, rpm);
	        			queueActuation(2, -rpm);
	        			queueActuation(3, 0);
	        			Thread.sleep(2000);
	        			diagonal = true;
						
					}
					else{
						queueActuation(0, -rpm);
	        			queueActuation(1, 0);
	        			queueActuation(2, 0);
	        			queueActuation(3, rpm);
	        			Thread.sleep(2000);
	        			diagonal = false;
					}
				}
			}
		} catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
	}

	@Override
	public void run() {
		try {
			runWithExceptions();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (TimeoutException e) {
			e.printStackTrace();
		}
	}

	private static Actuation queueActuation(int id, double targetValue){
        Actuation act = new Actuation(id, targetValue);
        System.out.println("Queueing Actuation for Motor ID: " + id + ", Target value: " + targetValue);
        hci.queueActuation(act);
        return act;
    }

	public AlternatingTurningController(double speed, HardwareControlInterface hci){
		this.hci = hci;
		this.rpm = Math.abs(speed);
		this.dir = speed > 0 ? true : false;
	}

}