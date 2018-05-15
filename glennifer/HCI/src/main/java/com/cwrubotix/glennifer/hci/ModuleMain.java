package com.cwrubotix.glennifer.hci;

import com.cwrubotix.glennifer.Messages;

import com.rabbitmq.client.*;
import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortList;
import jssc.SerialPortTimeoutException;
import org.yaml.snakeyaml.Yaml;

import java.io.FileInputStream;
import java.io.IOException;
import com.google.protobuf.InvalidProtocolBufferException;
import java.io.InputStream;
import java.util.Map;
import java.util.ArrayList;
import java.util.concurrent.TimeoutException;

public class ModuleMain {
	
    private static final int baud = 115200;
	/* Listen for Topics */
	//Motor Controls
	private static final String motorTopic = "motorcontrol.#";
    private static final String inputYMLPath = "config/connection.yml";
	
	private static HardwareControlInterface hci;
    private static String serverAddress;
    private static String serverUsername;
    private static String serverPassword;
    private static String exchangeName;
    private static Channel channel;
    private static String queueName;

	public static void runWithConnectionExceptions() throws IOException, TimeoutException {

        // Read connection config
		Mechanics.initialize();

        // Read String paths from YML file
		getVarsFromConfigFile(inputYMLPath);

		//Connect and Configure AMPQ
        setupAMQP();

        hci = new HardwareControlInterface();

		// Initialize sensors
        initializeSensors();  
                
        // Initialize actuators
        initializeActuators();
                	
		// Start HCI
		Thread hciThread = new Thread(hci);
		hciThread.start();

		//Start AMPQ Thread
		//Listen for messages

		channel.queueBind(queueName, exchangeName, motorTopic);

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

                } else if (keys[1].equals("looky")) {
                    routeLookyMessage(keys, body);
                }

                else if (keys[1].equals("system")){
                    routeSystemMessage(keys, body);
			    }
		    };
		};

		channel.basicConsume(queueName, true, consumer);
        // Main loop to get sensor data
        //generateDummyMessage();
        try {
            while (true) {
                //System.out.println("Looping in main");
                SensorData sensorData = hci.pollSensorUpdate();
                generateSensorUpdateMessage(sensorData);

                if (!hciThread.isAlive()) {
                    break;
                }
            }
        } catch (InterruptedException e) {}
    }

    private static void generateSensorUpdateMessage(SensorData sensorData) throws IOException{
        int sensorDataID =  sensorData.id;
        double value = sensorData.data;
        //System.out.print("data ID: " + sensorDataID);
        //System.out.println(", value: " + value);
        long time_ms = sensorData.timestamp;
        Messages.UnixTime unixTime = Messages.UnixTime.newBuilder()
                .setTimeInt(time_ms / 1000)
                .setTimeFrac((time_ms % 1000) / (1000.0F))
                .build();
        switch(sensorDataID){
            // LEFT WHEEL RPM
            case 1:
            case 7: {
                Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
                    .setRpm((float)value)
                    .setTimestamp(unixTime)
                    .build();
                if (sensorDataID == 1)
                    channel.basicPublish("amq.topic", "sensor.locomotion.front_left.wheel_rpm", null, msg.toByteArray());
                else if (sensorDataID == 7)
                    channel.basicPublish("amq.topic", "sensor.locomotion.back_left.wheel_rpm", null, msg.toByteArray());
                break;
            }

            // RIGHT WHEEL RPM
            case 3:
            case 5: {
                Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
                    .setRpm((float)value)
                    .setTimestamp(unixTime)
                    .build();
                if (sensorDataID == 3)
                    channel.basicPublish("amq.topic", "sensor.locomotion.front_right.wheel_rpm", null, msg.toByteArray());
                else if (sensorDataID == 5)
                    channel.basicPublish("amq.topic", "sensor.locomotion.back_right.wheel_rpm", null, msg.toByteArray());
                break;
            }

            // EXCAVATION ARM POSITION
            case 10:
            case 11: {
                value = (float)convertToBCAngle(value);
                Messages.PositionUpdate msg = Messages.PositionUpdate.newBuilder()
                    .setPosition((float)value)
                    .setTimestamp(unixTime)
                    .build();
                channel.basicPublish("amq.topic", "sensor.excavation.arm_pos", null, msg.toByteArray());
                break;
            }

            // EXCAVATION BC TRANSLATION DISPLACEMENT
            case 12: {
                value = (value - 2944) / 10.5;  
                Messages.DisplacementUpdate msg = Messages.DisplacementUpdate.newBuilder()
                    .setDisplacement((float)value)
                    .setTimestamp(unixTime)
                    .build();
                channel.basicPublish("amq.topic", "sensor.excavation.conveyor_translation_displacement", null, msg.toByteArray());
                
                break;
            }

            // EXCAVATION BC DIGGING CURRENT
            case 33: {
                Messages.CurrentUpdate msg = Messages.CurrentUpdate.newBuilder()
                    .setCurrent((float)value)
                    .setTimestamp(unixTime)
                    .build();
                channel.basicPublish("amq.topic", "sensor.excavation.conveyor_current", null, msg.toByteArray());
                break;
            }

            // DEPOSITION HOPPER RPM 
            // maybe it should be position controlled instead of speed controlled
            /*case 14: {
                Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
                    .setRpm((float)value)
                    .setTimestamp(unixTime)
                    .build();
                channel.basicPublish("amq.topic", "sensor.deposition.hopper_rpm", null, msg.toByteArray());
                break;
            }*/

            // LOAD CELLS
            case 22:
            case 23: {
                Messages.LoadUpdate msg = Messages.LoadUpdate.newBuilder()
                    .setLoad((float)value)
                    .setTimestamp(unixTime)
                    .build();
                if (sensorDataID == 22)
                    channel.basicPublish("amq.topic", "sensor.deposition.load.left", null, msg.toByteArray());
                else if (sensorDataID == 23)
                    channel.basicPublish("amq.topic", "sensor.deposition.load.right", null, msg.toByteArray());
                break;
            }

            // LIMIT SWITCHES
            case 13:
            case 14:
            case 15:
            case 16:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21: {
                Messages.LimitUpdate msg = Messages.LimitUpdate.newBuilder()
                    .setPressed(value > 0)
                    .setTimestamp(unixTime)
                    .build();
                if (sensorDataID == 13) 
                    channel.basicPublish("amq.topic", "sensor.excavation.conveyor_translation_limit_retracted", null, msg.toByteArray());
                else if (sensorDataID == 14)
                    channel.basicPublish("amq.topic", "sensor.excavation.conveyor_translation_limit_extended.right", null, msg.toByteArray());
                else if (sensorDataID == 15)
                    channel.basicPublish("amq.topic", "sensor.excavation.conveyor_translation_limit_extended.left", null, msg.toByteArray());
                else if (sensorDataID == 16)
                    channel.basicPublish("amq.topic", "sensor.excavation.arm_limit_extended.left", null, msg.toByteArray());
                else if (sensorDataID == 17)
                    channel.basicPublish("amq.topic", "sensor.excavation.arm_limit_extended.right", null, msg.toByteArray());
                else if (sensorDataID == 21)
                    channel.basicPublish("amq.topic", "sensor.deposition.hopper_limit_extended.left", null, msg.toByteArray());
                else if (sensorDataID == 18)
                    channel.basicPublish("amq.topic", "sensor.deposition.hopper_limit_extended.right", null, msg.toByteArray());
                else if (sensorDataID == 19)
                    channel.basicPublish("amq.topic", "sensor.deposition.hopper_limit_retracted.left", null, msg.toByteArray());
                else if (sensorDataID == 20)
                    channel.basicPublish("amq.topic", "sensor.deposition.hopper_limit_retracted.right", null, msg.toByteArray());
            }
        }
    }

    private static void getVarsFromConfigFile(String path) throws RuntimeException, IOException{
        InputStream input = new FileInputStream(path);
        Yaml yaml = new Yaml();
        Object connectionConfigObj = yaml.load(input);
        Map<String, String> connectionConfig = (Map<String, String>)connectionConfigObj;

        serverAddress = connectionConfig.get("server-addr");
        if (serverAddress == null) {
            throw new RuntimeException("Config file missing server-addr");
        }
        serverUsername = connectionConfig.get("server-user");
        if (serverUsername == null) {
            throw new RuntimeException("Config file missing server-user");
        }
        serverPassword = connectionConfig.get("server-pass");
        if (serverPassword == null) {
            throw new RuntimeException("Config file missing server-pass");
        }
        exchangeName = connectionConfig.get("exchange-name");
        if (exchangeName == null) {
            throw new RuntimeException("Config file missing exchange-name");
        }
    }

    private static void setupAMQP() throws IOException, TimeoutException{
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost(serverAddress); //replace local host with host name
        factory.setUsername(serverUsername);
        factory.setPassword(serverPassword);
        Connection connection = factory.newConnection(); // throws
        channel = connection.createChannel(); // throws
        queueName = channel.queueDeclare().getQueue();
    }

    private static void initializeSensors(){
        
        ArrayList<SensorConfig> sensorList = new ArrayList<SensorConfig>();
        
        // Locomotion 
        sensorList.add(new SensorConfig("Front Left Wheel Encoder", 1));
        sensorList.add(new SensorConfig("Front Right Wheel Encoder", 3));
        sensorList.add(new SensorConfig("Back Left Wheel Encoder", 7));
        sensorList.add(new SensorConfig("Back Right Wheel Encoder", 5));
        
        //Excavation
        sensorList.add(new SensorConfig("Left Arm Pot", 10));
        sensorList.add(new SensorConfig("Right Arm Pot", 11));
        sensorList.add(new SensorConfig("Left Arm Extended Limit", 16));
        sensorList.add(new SensorConfig("Right Arm Extended Limit", 17));
        sensorList.add(new SensorConfig("Bucket Conveyor Translation Pot", 12));
        sensorList.add(new SensorConfig("Bucket Conveyor Retracted Limit", 13));
        sensorList.add(new SensorConfig("Bucket Conveyor Extended Limit A", 15));
        sensorList.add(new SensorConfig("Bucket Conveyor Extended Limit B", 14));
        sensorList.add(new SensorConfig("Bucket Conveyor Current", 33));

        //Deposition
        sensorList.add(new SensorConfig("Load Cell A", 22));
        sensorList.add(new SensorConfig("Load Cell B", 23));
        sensorList.add(new SensorConfig("Hopper Extended Limit A", 21));
        sensorList.add(new SensorConfig("Hopper Extended Limit B", 18));
        sensorList.add(new SensorConfig("Hopper Retracted Limit A", 19));
        sensorList.add(new SensorConfig("Hopper Retracted Limit B", 20));

        // Add sensors
        for (SensorConfig config : sensorList){
            hci.addSensor(config);
        }
    }

    private static void initializeActuators(){
        ArrayList<ActuatorConfig> actuatorList = new ArrayList<ActuatorConfig>();

        actuatorList.add(new ActuatorConfig("Front Left Drive Motor", 0));
        actuatorList.add(new ActuatorConfig("Front Right Drive Motor", 1));
        actuatorList.add(new ActuatorConfig("Back Left Drive Motor", 2));
        actuatorList.add(new ActuatorConfig("Back Right Drive Motor", 3));
        actuatorList.add(new ActuatorConfig("Bucket Conveyor Digging Motor", 4));
        actuatorList.add(new ActuatorConfig("Deposition Motor", 5));
        actuatorList.add(new ActuatorConfig("Left Arm Actuator", 6));
        actuatorList.add(new ActuatorConfig("Right Arm Actuator", 7));
        actuatorList.add(new ActuatorConfig("Bucket Conveyor Translation Motor Position", 8));
        actuatorList.add(new ActuatorConfig("Left Looky Servo", 9));
        actuatorList.add(new ActuatorConfig("Right Looky Servo", 10));
        actuatorList.add(new ActuatorConfig("Bucket Conveyor Translation Motor Speed", 11));

         // Add sensors
        for (ActuatorConfig config : actuatorList){
            hci.addActuator(config);
        }
    }

    private static void generateDummyMessage() throws InvalidProtocolBufferException, IOException{
        Messages.SpeedControlCommand msg = Messages.SpeedControlCommand.newBuilder()
                            .setRpm(-2.0f)
                            .setTimeout(1000f)
                            .build();
        channel.basicPublish("amq.topic", "motorcontrol.locomotion.front_left.wheel_rpm", null, msg.toByteArray());
        System.out.println("sent dummy wheel rpm message");
    }

    private static void routeLocomotionMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        
        if(keys.length < 4) {
            System.out.println("Locomotion motor control routing key must have 4 elements");
            return;
        }

        if (keys[3].equals("wheel_rpm")) {
            routeWheelRPMMessage(keys, body);
        } else if(keys[3].equals("turn")) {
            routeTurnMessage(keys, body);
        } else if(keys[3].equals("all_wheels")) {
            routeAllWheelsMessage(keys, body);
        } else {
            System.out.println("Locomotion motor control routing key has unrecognized motor");
            return;
        }
    }

    private static void routeWheelRPMMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        int id = -1;
        if (keys[2].equals("front_left")) {
            id = 0;
        } else if (keys[2].equals("front_right")) {
            id = 1;
        } else if (keys[2].equals("back_left")) {
            id = 2;
        } else if (keys[2].equals("back_right")) {
            id = 3;
        } else {
            System.out.println("Locomotion motor control routing key has invalid wheel");
            return;
        }
        double targetValue = 0;
        Messages.SpeedControlCommand scc = Messages.SpeedControlCommand.parseFrom(body);
        targetValue = (scc.getRpm() * 2);
        if(id == 0){
            targetValue *= 1.00;
        }
        if(id == 1){
            targetValue *= 1.00;
        }
        if(id == 2){
            targetValue *= 1.00;
        }
        if(id == 3){
            targetValue *= 1.00;
        }
        queueActuation(id, targetValue);
    }

    private static void routeTurnMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        Messages.TurnControlCommand tc = Messages.TurnControlCommand.parseFrom(body);
        int id0 = 0;
        int id1 = 1;
        int id2 = 2;
        int id3 = 3;
        double targetValue0 = 0;
        double targetValue1 = 0;
        double targetValue2 = 0;
        double targetValue3 = 0;

        //idk how to do this transformation yet
        if(tc.getCurvature() > 0){
            targetValue0 = tc.getSpeed();
            targetValue2 = tc.getSpeed();

            targetValue1 = tc.getSpeed() * tc.getCurvature();
            targetValue3 = tc.getSpeed() * tc.getCurvature();
        }
        else{
            targetValue0 = tc.getSpeed() * tc.getCurvature();
            targetValue2 = tc.getSpeed() * tc.getCurvature();

            targetValue1 = tc.getSpeed();
            targetValue3 = tc.getSpeed();
        }

        queueActuation(id0, targetValue0);
        queueActuation(id1, targetValue1);
        queueActuation(id2, targetValue2);
        queueActuation(id3, targetValue3);
    }

    private static void routeAllWheelsMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        Messages.AllWheelsCommand awc = Messages.AllWheelsCommand.parseFrom(body);
        int id0 = 0;
        int id1 = 1;
        int id2 = 2;
        int id3 = 3;
        double targetValue0 = awc.getFrontLeft();
        double targetValue1 = awc.getFrontRight();
        double targetValue2 = awc.getBackLeft();
        double targetValue3 = awc.getBackRight();

        queueActuation(id0, targetValue0);
        queueActuation(id1, targetValue1);
        queueActuation(id2, targetValue2);
        queueActuation(id3, targetValue3);
    }

    private static void routeExcavationMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        if(keys.length < 3) {
            System.out.println("Excavation motor control routing key must have 3 elements");
            return;
        }

        if (keys[2].equals("conveyor_translation_displacement")) {
            Messages.PositionControlCommand pcc = Messages.PositionControlCommand.parseFrom(body);
            int id = 8;
            double targetValue = pcc.getPosition() * 10;
            queueActuation(id, targetValue);      
        } else if (keys[2].equals("conveyor_translation_speed")) {
            Messages.SpeedControlCommand pcc = Messages.SpeedControlCommand.parseFrom(body);
            int id = 11;
            double targetValue = pcc.getRpm() * 5;
            queueActuation(id, targetValue);
        }else if (keys[2].equals("arm_pos")) {
            Messages.PositionControlCommand pcc = Messages.PositionControlCommand.parseFrom(body);
            int id1 = 6;
            int id2 = 7;
            double targetValue = pcc.getPosition() * 10;
            if(targetValue == 1000){
                targetValue = 2000;
            }
            queueActuation(id1, targetValue);
            queueActuation(id2, targetValue);
        } else if (keys[2].equals("bucket_conveyor_rpm")) {
            Messages.SpeedControlCommand scc = Messages.SpeedControlCommand.parseFrom(body);
            int id = 4;
            double targetValue = scc.getRpm() * 4;
            queueActuation(id, targetValue);
        } else {
            System.out.println("Excavation motor control routing key has unrecognized motor");
            return;
        }
    }

    private static void routeDepositionMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        if(keys.length < 3) {
            System.out.println("Deposition motor control routing key must have 3 elements");
            return;
        }
        if (keys[2].equals("dump_pos")) {
            Messages.PositionControlCommand pcc = Messages.PositionControlCommand.parseFrom(body);
            int id = 5;
            double targetValue = pcc.getPosition() * 2;
            queueActuation(id, targetValue);
        } else {
            System.out.println("Deposition motor control routing key has unrecognized motor");
            return;
        }
    }

    private static void routeLookyMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        if(keys.length < 4) {
            System.out.println("Looky motor control routing key must have 4 elements");
            return;
        }
        if (keys[2].equals("turn")) {
            int id;
            Messages.PositionControlCommand pcc = Messages.PositionControlCommand.parseFrom(body);
            double targetValue = pcc.getPosition();
            if(keys[3].equals("left")){
                id = 9;
                targetValue -= 270;
            }
            else if(keys[3].equals("right")){
                id = 10;
                targetValue -= 90;
            }
            else{
                System.out.println("Invalid looky motor side");
                return;
            }
            queueActuation(id, targetValue);
        } else {
            System.out.println("Looky motor control routing key has unrecognized motor");
            return;
        }
    }

    private static void routeSystemMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        if(keys[2].equals("stop_all")){
            Messages.StopAllCommand sac = Messages.StopAllCommand.parseFrom(body);
            int id = 50; //the "50th motor tells all motors to stop or start"
            double targetValue = 0;
            if(sac.getStop() == true){
                targetValue = 0;
            }
            else {
                targetValue = 1;
            }
            System.out.println("Stop All command issued");
            queueActuation(id, targetValue);
        }
        else {
        System.out.println("Motor control routing key has unrecognized subsystem");
        return;
        }
    }

    private static Actuation queueActuation(int id, double targetValue){
        Actuation act = new Actuation(id, targetValue);
        System.out.println("Queueing Actuation for Motor ID: " + id + ", Target value: " + targetValue);
        hci.queueActuation(act);
        return act;
    }

	private static int sign(double x) {
		if (x > 0) {
			return 1;
		} else if (x < 0) {
			return -1;
		} else {
			return 0;
		}
	}
	
	/**
	 * Takes Voltage read from BC arm actuators and turn it into the BC angle position
	 * If this method returns 0, the BC is horizontal to the ground.
	 * If this method returns 90, the BC is vertical to the ground.
	 * @param voltage
	 * @return the angle position of BC in degrees.
	 */
	private static double convertToBCAngle(double voltage){
		/*All the magic numbers are measured in SolidWorks assuming and setting the extension length and bc angle
		 * are both 0 when BC is horizontal to the ground.*/
		double C = 48.7892 * Math.PI / 180;
		double a = 3.23433;
		double b = 0.37656 + (voltage - 0.04624)/0.79547; // Paul's equation
		double c = Math.sqrt(a*a+ b*b - 2 * a * b * Math.cos(C));
		double rad =  Math.acos((b * b + c * c - a * a)/(2 * b * c)) - (5.10922 * Math.PI / 180);
		return rad * 180 / Math.PI;
	}

	public static void main(String[] args) {
		try {
			runWithConnectionExceptions();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (TimeoutException e) {
			e.printStackTrace();
		}



	}

}
