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
	
    private static final int baud = 9600;
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

        String arduinoPort = findArduinoPort();
        hci = createHCIFromPort(arduinoPort);

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

                } else if (keys[1].equals("system")){
                    routeSystemMessage(keys, body);
			    }
		    };
		};

		channel.basicConsume(queueName, true, consumer);
        // Main loop to get sensor data
        generateDummyMessage();
        try {
            while (true) {
                System.out.println("Looping in main");
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
        long time_ms = sensorData.timestamp;
        Messages.UnixTime unixTime = Messages.UnixTime.newBuilder()
                .setTimeInt(time_ms / 1000)
                .setTimeFrac((time_ms % 1000) / (1000.0F))
                .build();
        switch(sensorDataID){
            // LEFT WHEEL RPM
            case 0:
            case 2: {
                value = -(Mechanics.wheelValueToRPM(value));
                Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
                    .setRpm((float)value)
                    .setTimestamp(unixTime)
                    .build();
                if (sensorDataID == 0)
                    channel.basicPublish("amq.topic", "sensor.locomotion.front_left.wheel_rpm", null, msg.toByteArray());
                else if (sensorDataID == 2)
                    channel.basicPublish("amq.topic", "sensor.locomotion.back_left.wheel_rpm", null, msg.toByteArray());
                break;
            }

            // RIGHT WHEEL RPM
            case 1:
            case 3: {
                value = (Mechanics.wheelValueToRPM(value));
                Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
                    .setRpm((float)value)
                    .setTimestamp(unixTime)
                    .build();
                if (sensorDataID == 1)
                    channel.basicPublish("amq.topic", "sensor.locomotion.front_right.wheel_rpm", null, msg.toByteArray());
                else if (sensorDataID == 3)
                    channel.basicPublish("amq.topic", "sensor.locomotion.back_right.wheel_rpm", null, msg.toByteArray());
                break;
            }

            // EXCAVATION ARM POSITION
            case 4:
            case 5: {
                value = (float)convertToBCAngle(value);
                Messages.PositionUpdate msg = Messages.PositionUpdate.newBuilder()
                    .setPosition((float)value)
                    .setTimestamp(unixTime)
                    .build();
                channel.basicPublish("amq.topic", "sensor.excavation.arm_pos", null, msg.toByteArray());
                break;
            }

            // EXCAVATION BC TRANSLATION DISPLACEMENT
            case 8: {
                Messages.DisplacementUpdate msg = Messages.DisplacementUpdate.newBuilder()
                    .setDisplacement((((float)value - 45.0F) / 785.0F) * 100.0F)
                    .setTimestamp(unixTime)
                    .build();
                channel.basicPublish("amq.topic", "sensor.excavation.bucket_conveyor_translation_displacement", null, msg.toByteArray());
                break;
            }

            // EXCAVATION BC DIGGING CURRENT
            case 13: {
                Messages.CurrentUpdate msg = Messages.CurrentUpdate.newBuilder()
                    .setCurrent((float)value / 100.0F)
                    .setTimestamp(unixTime)
                    .build();
                channel.basicPublish("amq.topic", "sensor.excavation.conveyor_current", null, msg.toByteArray());
                break;
            }

            // DEPOSITION HOPPER RPM 
            // maybe it should be position controlled instead of speed controlled
            case 14: {
                Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
                    .setRpm((float)value)
                    .setTimestamp(unixTime)
                    .build();
                channel.basicPublish("amq.topic", "sensor.deposition.hopper_rpm", null, msg.toByteArray());
                break;
            }

            // LOAD CELLS
            case 15:
            case 16: {
                Messages.LoadUpdate msg = Messages.LoadUpdate.newBuilder()
                    .setLoad((float)value)
                    .setTimestamp(unixTime)
                    .build();
                if (sensorDataID == 15)
                    channel.basicPublish("amq.topic", "sensor.deposition.load.left", null, msg.toByteArray());
                else if (sensorDataID == 16)
                    channel.basicPublish("amq.topic", "sensor.deposition.load.right", null, msg.toByteArray());
                break;
            }

            // LIMIT SWITCHES
            case 6:
            case 7:
            case 9:
            case 10:
            case 11:
            case 12:
            case 17:
            case 18:
            case 19:
            case 20: {
                Messages.LimitUpdate msg = Messages.LimitUpdate.newBuilder()
                    .setPressed(value > 0)
                    .setTimestamp(unixTime)
                    .build();
                if (sensorDataID == 6)
                    channel.basicPublish("amq.topic", "sensor.excavation.arm_limit_extended.left", null, msg.toByteArray());
                else if (sensorDataID == 7)
                    channel.basicPublish("amq.topic", "sensor.excavation.arm_limit_extended.right", null, msg.toByteArray());
                else if (sensorDataID == 9)
                    channel.basicPublish("amq.topic", "sensor.excavation.bucket_conveyor_translation_limit_extended.left", null, msg.toByteArray());
                else if (sensorDataID == 10)
                    channel.basicPublish("amq.topic", "sensor.excavation.bucket_conveyor_translation_limit_extended.right", null, msg.toByteArray());
                else if (sensorDataID == 11)
                    channel.basicPublish("amq.topic", "sensor.excavation.bucket_conveyor_translation_limit_retracted.left", null, msg.toByteArray());
                else if (sensorDataID == 12)
                    channel.basicPublish("amq.topic", "sensor.excavation.bucket_conveyor_translation_limit_retracted.right", null, msg.toByteArray());
                else if (sensorDataID == 17)
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

    private static String findArduinoPort(){
        // Initialize port as null
        String port = null;
        // For each attached serial port
        for(String s:SerialPortList.getPortNames()) {
            SerialPort sp = new SerialPort(s);
            try {
                // Open the port
                sp.openPort();
                Thread.sleep(1000);
                sp.setParams(baud, 8, 1, 0);
                sp.setDTR(false);
                // Create test packet
                byte[] bt = {0x03,0x01,0x5A};
                // Write test byte 0x5A
                sp.writeBytes(bt);
                System.out.println(bt[2]);
                Thread.sleep(1000);
                // Read response bytes
                byte[] b = sp.readBytes(3,2000);
                System.out.println(b[2]);
                // If response is 0xA5, it is the arduino
                if(b[2] == (byte)0xA5) {
                	System.out.println("Found the arduino");
                    // Capture the string of correct port
                    port = s;
                    // Close the port
                    sp.closePort();
                    break;
                }
                sp.closePort();
            } catch(SerialPortException e) {
                e.printStackTrace();
                continue;
            } catch(SerialPortTimeoutException e) {
                try {
                        sp.closePort();
                } catch(Exception e1) {
                        e1.printStackTrace();
                }
                continue;
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        return port;
    }

    private static HardwareControlInterface createHCIFromPort(String port){
        // If port is still null, couldn't find it
        if(port == null) {
            System.out.println("Couldn't find attached arduino, please try again");
            return null;
        } else {
            System.out.println("Found arduino at " + port);
        }
        return new HardwareControlInterface(port);           
    }

    private static void initializeSensors(){
        
        ArrayList<SensorConfig> sensorList = new ArrayList<SensorConfig>();
        
        // Locomotion 
        sensorList.add(new SensorConfig("Front Left Wheel Encoder", 0));
        sensorList.add(new SensorConfig("Front Right Wheel Encoder", 1));
        sensorList.add(new SensorConfig("Back Left Wheel Encoder", 2));
        sensorList.add(new SensorConfig("Back Right Wheel Encoder", 3));
        
        //Excavation
        sensorList.add(new SensorConfig("Left Arm Pot", 4));
        sensorList.add(new SensorConfig("Right Arm Pot", 5));
        sensorList.add(new SensorConfig("Left Arm Extended Limit", 6));
        sensorList.add(new SensorConfig("Right Arm Extended Limit", 7));
        sensorList.add(new SensorConfig("Bucket Conveyor Translation Pot", 8));
        sensorList.add(new SensorConfig("Bucket Conveyor Extended Limit A", 9));
        sensorList.add(new SensorConfig("Bucket Conveyor Extended Limit B", 10));
        sensorList.add(new SensorConfig("Bucket Conveyor Retracted Limit A", 11));
        sensorList.add(new SensorConfig("Bucket Conveyor Retracted Limit B", 12));
        sensorList.add(new SensorConfig("Bucket Conveyor Current", 13));

        //Deposition
        sensorList.add(new SensorConfig("Hopper Encoder", 14));
        sensorList.add(new SensorConfig("Load Cell A", 15));
        sensorList.add(new SensorConfig("Load Cell B", 16));
        sensorList.add(new SensorConfig("Hopper Extended Limit A", 17));
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
        actuatorList.add(new ActuatorConfig("Left Arm Actuator", 4));
        actuatorList.add(new ActuatorConfig("Right Arm Actuator", 5));
        actuatorList.add(new ActuatorConfig("Bucket Conveyor Translation Motor", 6));
        actuatorList.add(new ActuatorConfig("Bucket Conveyor Digging Motor", 7));        
        actuatorList.add(new ActuatorConfig("Deposition Motor", 8));

         // Add sensors
        for (ActuatorConfig config : actuatorList){
            hci.addActuator(config);
        }
    }

    private static void generateDummyMessage() throws InvalidProtocolBufferException, IOException{
        Messages.SpeedControlCommand msg = Messages.SpeedControlCommand.newBuilder()
                            .setRpm(1000f)
                            .setTimeout(1000f)
                            .build();
        channel.basicPublish("amq.topic", "motorcontrol.locomotion.front_left.wheel_rpm", null, msg.toByteArray());
        System.out.println("sent wheel rpm message");
    }

    private static void routeLocomotionMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        if(keys.length < 4) {
            System.out.println("Locomotion motor control routing key must have 4 elements");
            return;
        }

        if (keys[3].equals("wheel_rpm")) {
            routeWheelRPMMessage(keys, body);

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
        if (id % 2 == 0) { //Left wheels need reversed directions
            targetValue = -scc.getRpm();
        } else {
            targetValue = scc.getRpm();
        }
        queueActuation(id, targetValue);
    }

    private static void routeExcavationMessage(String[] keys, byte[] body) throws InvalidProtocolBufferException{
        if(keys.length < 3) {
            System.out.println("Excavation motor control routing key must have 3 elements");
            return;
        }

        if (keys[2].equals("conveyor_translation_displacement")) {
            Messages.PositionControlCommand pcc = Messages.PositionControlCommand.parseFrom(body);
            int id = 6;
            double targetValue = ((pcc.getPosition() / 100.0F) * 785) + 45;
            queueActuation(id, targetValue);
        } else if (keys[2].equals("arm_pos")) {
            Messages.PositionControlCommand pcc = Messages.PositionControlCommand.parseFrom(body);
            int id1 = 4;
            int id2 = 5;
            double targetValue = ((90- pcc.getPosition()) * 5.555) + 100 ;
            queueActuation(id1, targetValue);
            queueActuation(id2, targetValue);
        } else if (keys[2].equals("bucket_conveyor_rpm")) {
            Messages.SpeedControlCommand scc = Messages.SpeedControlCommand.parseFrom(body);
            int id = 7;
            double targetValue = (scc.getRpm() / 100.0F) * 32767;
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
            int id = 8;
            double targetValue = (pcc.getPosition() / 100.0F) * 127;
            queueActuation(id, targetValue);
        } else {
            System.out.println("Deposition motor control routing key has unrecognized motor");
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
