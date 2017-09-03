package com.cwrubotix.glennifer.motor_dispatch;

/* this class control receive commands from path planning 
 * and configure glennifer motors and actuators accordingly
 */

import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Consumer;
import com.rabbitmq.client.DefaultConsumer;


public class MotorLayer {
  /* declare motor indentity */
  Motor [] wheel = new Motor[4];        // declare 4 wheel objects
  Motor bucket = new Motor();           // declare bucket object
  Motor conveyor = new Motor();         // declare conveyor object

  Pod [] wheelPod = new Pod[4];         // declare 4 wheel pod objects
  Pod excavation = new Pod();           // declare excavation pod object
  Pod dump = new Pod();                 // declare dump pod object
  
   
  private final static String QUEUE_NAME = "motorhighlevel";

  public static void main (String[] args) 
  throws java.io.IOException, java.lang.InterruptedException {
    ConnectionFactory factory = new ConnectionFactory();
    factory.sethost("localhost");
    Connection connection = factory.newConnection();
    Channel channel = connection.createChannel();

    channel.queueDeclare(QUEUE_NAME, true, false, false, null);
    System.out.println("Waiting for message. CTRL+C to exit");

    Consumer consumer = new DefaultConsumer(channel) {
    @Override
    public void handleDelivery (String consumerTag, Envelope envelope, 
    AMQP.BasicProperties properties, byte[] body) throws IOException {
      String message = new String (body, "UTF-8");
      }
    }
  }
}
