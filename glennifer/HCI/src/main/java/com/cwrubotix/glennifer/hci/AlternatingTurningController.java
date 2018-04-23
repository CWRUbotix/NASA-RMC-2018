package com.cwrubotix.glennifer.hci;

import com.cwrubotix.glennifer.Messages;
import com.rabbitmq.client.*;
import com.google.protobuf.InvalidProtocolBufferException;

public class HardwareControlInterface implements Runnable {

	private static final String motorTopic = "motorcontrol.#";
	private static String exchangeName = "amq.topic";
    private static Channel channel;
    private static String queueName;

	HardwareControlInterface hci;
	double rpm;
	boolean dir; //true is right, false is left
	boolean canContinue = true;

	@Override
	public void run() {

		Consumer consumer = new DefaultConsumer(channel) {
			@Override
			public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
				String routingKey = envelope.getRoutingKey();
				String[] keys = routingKey.split("\\.");
                if(keys.length < 2) {
					System.out.println("Motor control routing key must have a second element");
					return;
				}
				if (keys[1].equals("locomotion")) {
                    routeLocomotionMessage(keys, body);

                } else if (keys[1].equals("excavation")) {
                    routeExcavationMessage(keys, body);

                } else if (keys[1].equals("deposition")) {
                    routeDepositionMessage(keys, body);

                } else if (keys[1].equals("system")){
                    routeSystemMessage(keys, body);
			    }
		    };
		};

		while(canContinue){

		}
	}

	public HardwareControlInterface(double speed, HardwareControlInterface hci, Channel channel){
		this.hci = hci;
		this.rpm = Math.abs(speed);
		this.dir = speed > 0 ? true : false;
		this.channel = channel;
	}

}