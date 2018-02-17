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
    private static final String serverAddress;
    private static final String serverUsername;
    private static final String serverPassword;
    private static final String exchangeName;
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
		try {
			while (true) {
				System.out.println("Looping in main")
				SensorData sensorData = hci.pollSensorUpdate();
				
				int sensorDataID =  sensorData.id;
				double value = sensorData.data;
				long time_ms = sensorData.timestamp;
				Messages.UnixTime unixTime = Messages.UnixTime.newBuilder()
						.setTimeInt(time_ms / 1000)
						.setTimeFrac((time_ms % 1000) / (1000.0F))
						.build();

                        
                //LEFT WHEEL RPM    
				if (sensorDataID == 0 || sensorDataID == 2){
                    value = -(Mechanics.wheelValueToRPM(value));
					Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
							.setRpm((float)value)
							.setTimestamp(unixTime)
							.build();
                    if (sensorDataID == 0)
                        channel.basicPublish("amq.topic", "sensor.locomotion.front_left.wheel_rpm", null, msg.toByteArray());
                    else if (sensorDataID == 2)
                        channel.basicPublish("amq.topic", "sensor.locomotion.back_left.wheel_rpm", null, msg.toByteArray());
				} 
                //RIGHT WHEEL RPM
                else if (sensorDataID == 1 || sensorDataID == 3){
                    value = Mechanics.wheelValueToRPM(value);
					Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
							.setRpm((float)value)
							.setTimestamp(unixTime)
							.build();
                    if (sensorDataID == 1)        
					    channel.basicPublish("amq.topic", "sensor.locomotion.front_right.wheel_rpm", null, msg.toByteArray());
                    else if (sensorDataID == 3)
                        channel.basicPublish("amq.topic", "sensor.locomotion.back_right.wheel_rpm", null, msg.toByteArray());
				}
                //WHEEL POD POSITION
                else if (sensorDataID == 4 || sensorDataID == 5 || sensorDataID == 6 || sensorDataID == 7){
                    value = Mechanics.wheelPodValueToPos(value);
					if (value > 1) value = 1;
					if (value < -1) value = -1;
					value = Mechanics.wheelPosToRad(value);
					Messages.PositionUpdate msg = Messages.PositionUpdate.newBuilder()
							.setPosition((float)value)
							.setTimestamp(unixTime)
							.build();
                    if (sensorDataID == 4)
                        channel.basicPublish("amq.topic", "sensor.locomotion.front_left.wheel_pod_pos", null, msg.toByteArray());
                    else if (sensorDataID == 5)
                        channel.basicPublish("amq.topic", "sensor.locomotion.front_right.wheel_pod_pos", null, msg.toByteArray());
                    else if (sensorDataID == 6)
                        channel.basicPublish("amq.topic", "sensor.locomotion.back_left.wheel_pod_pos", null, msg.toByteArray());
                    else if (sensorDataID == 7)
                        channel.basicPublish("amq.topic", "sensor.locomotion.back_right.wheel_pod_pos", null, msg.toByteArray());
                }
                //WHEEL POD LIMIT EXTENDED
                /*
                else if (sensorDataID == 5 || sensorDataID == 8 || sensorDataID == 11 || sensorDataID == 14){
                    Messages.LimitUpdate msg = Messages.LimitUpdate.newBuilder()
                            .setPressed();
                }
                */
                // BC limits
                else if (sensorDataID == 23 || sensorDataID == 24 || sensorDataID == 25 || sensorDataID == 26) {
                    Messages.LimitUpdate msg = Messages.LimitUpdate.newBuilder()
                            .setPressed(value > 0)
                            .setTimestamp(unixTime)
                            .build();
                    if (sensorDataID == 23) {
                        channel.basicPublish("amq.topic", "sensor.excavation.conveyor_translation_limit_retracted.left", null, msg.toByteArray());
                    }
                    else if (sensorDataID == 24) {
                        channel.basicPublish("amq.topic", "sensor.excavation.conveyor_translation_limit_extended.left", null, msg.toByteArray());
                    }
                    else if (sensorDataID == 25) {
                        channel.basicPublish("amq.topic", "sensor.excavation.conveyor_translation_limit_retracted.right", null, msg.toByteArray());
                    }
                    else if (sensorDataID == 26) {
                        channel.basicPublish("amq.topic", "sensor.excavation.conveyor_translation_limit_extended.right", null, msg.toByteArray());
                    }
                }
                // sensor.locomotion.front_left.wheel_pod_limit_extended
				// sensor.locomotion.front_right.wheel_pod_limit_extended
				// sensor.locomotion.back_left.wheel_pod_limit_extended
				// sensor.locomotion.back_right.wheel_pod_limit_extended
				else if(sensorDataID == 16){
					Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
							.setRpm((float)convertToBCAngle(value))
							.setTimestamp(unixTime)
							.build();
					channel.basicPublish("amq.topic","sensor.excavation.arm_pos_a", null, msg.toByteArray());
				} else if(sensorDataID == 19){
					Messages.RpmUpdate msg = Messages.RpmUpdate.newBuilder()
							.setRpm((float)convertToBCAngle(value))
							.setTimestamp(unixTime)
							.build();
					channel.basicPublish("amq.topic", "sensor.excavation.arm_pos_b", null, msg.toByteArray());
				} else if(sensorDataID == 22){
					Messages.PositionUpdate msg = Messages.PositionUpdate.newBuilder()
							.setPosition((((float)value - 45.0F) / 785.0F) * 100.0F)
							.setTimestamp(unixTime)
							.build();
					channel.basicPublish("amq.topic","sensor.excavation.translation_pos", null,msg.toByteArray());
                } else if(sensorDataID == 38) { // Excavation conveyor current
                    Messages.CurrentUpdate msg = Messages.CurrentUpdate.newBuilder()
                            .setCurrent((float)value / 100.0F)
                            .setTimestamp(unixTime)
                            .build();
                    channel.basicPublish("amq.topic","sensor.excavation.conveyor_current", null,msg.toByteArray());
				} else {
					// TODO: do others
				}
				if (!hciThread.isAlive()) {
				    break;
                }
			}
		} catch (InterruptedException e) { }
	}


    private static void getVarsFromConfigFile(String path) throws RuntimeException{
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

    private static void setupAMQP() throws IOException{
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
                sp.setParams(baud, 8, 1, 0);
                sp.setDTR(false);
                // Create test packet
                byte[] bt = {0x02,0x01,0x5A};
                // Write test byte 0x5A
                sp.writeBytes(bt);
                Thread.sleep(1000);
                // Read response bytes
                byte[] b = sp.readBytes(3,1000);
                // If response is 0xA5, it is the arduino
                if(b[2] == (byte)0xA5) {
                	System.out.println("Found the arduino")
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
        sensorList.add(new SensorConfig("Front Left Wheel Pod Position", 4));
        sensorList.add(new SensorConfig("Front Left Wheel Pod Limit 90", 5));
        sensorList.add(new SensorConfig("Front Left Wheel Pod Limit 0", 6));
        sensorList.add(new SensorConfig("Front Right Wheel Pod Position", 7));
        sensorList.add(new SensorConfig("Front Right Wheel Pod Limit 90", 8));
        sensorList.add(new SensorConfig("Front Right Wheel Pod Limit 0", 9));
        sensorList.add(new SensorConfig("Back Left Wheel Pod Position", 10));
        sensorList.add(new SensorConfig("Back Left Wheel Pod Limit 90", 11));
        sensorList.add(new SensorConfig("Back Left Wheel Pod Limit 0", 12));
        sensorList.add(new SensorConfig("Back Right Wheel Pod Position", 13));
        sensorList.add(new SensorConfig("Back Right Wheel Pod Limit 90", 14));
        sensorList.add(new SensorConfig("Back Right Wheel Pod Limit 0", 15));

        //Excavation
        sensorList.add(new SensorConfig("Arm A Angle Pot Position", 16));
        sensorList.add(new SensorConfig("Arm A Limit Low", 17));
        sensorList.add(new SensorConfig("Arm A Limit High", 18));
        sensorList.add(new SensorConfig("Arm B Angle Pot Position", 19));
        sensorList.add(new SensorConfig("Arm B Limit Low", 20));
        sensorList.add(new SensorConfig("Arm B Limit High", 21));
        sensorList.add(new SensorConfig("Translation Pot Position", 22));
        sensorList.add(new SensorConfig("Translation Pot A Limit Low", 23));
        sensorList.add(new SensorConfig("Translation Pot A Limit High", 24));
        sensorList.add(new SensorConfig("Translation Pot B Limit Low", 25));
        sensorList.add(new SensorConfig("Translation Pot B Limit High", 26));
        sensorList.add(new SensorConfig("Belt Encoder Speed", 27));
        sensorList.add(new SensorConfig("Excavation Bucket Conveyor Current", 38));

        //Deposition
        sensorList.add(new SensorConfig("Load Cell A", 28));
        sensorList.add(new SensorConfig("Load Cell B", 29));
        sensorList.add(new SensorConfig("Load Cell C", 30));
        sensorList.add(new SensorConfig("Load Cell D", 31));
        sensorList.add(new SensorConfig("Deposition Actuator Pot A Position", 32));
        sensorList.add(new SensorConfig("Deposition Actuator Pot A Limit Low", 33));
        sensorList.add(new SensorConfig("Deposition Actuator Pot A Limit High", 34));
        sensorList.add(new SensorConfig("Deposition Actuator Pot B Position", 35));
        sensorList.add(new SensorConfig("Deposition Actuator Pot B Limit Low", 36));
        sensorList.add(new SensorConfig("Deposition Actuator Pot B Limit High", 37));

        // Add sensors
        for (SensorConfig config : sensorList){
            hci.addSensor(config);
        }
    }

    private static void initializeActuators(){
        ArrayList<ActuatorConfig> actuatorList = new ArrayList<ActuatorConfig>();

        actuatorList.add(new ActuatorConfig("Left Rear Drive Motor", 0));
        actuatorList.add(new ActuatorConfig("Right Rear Drive Motor", 1));
        actuatorList.add(new ActuatorConfig("Left Front Drive Motor", 2));
        actuatorList.add(new ActuatorConfig("Right Front Drive Motor", 3));
        actuatorList.add(new ActuatorConfig("Front Left Turning Actuator", 4));
        actuatorList.add(new ActuatorConfig("Front Right Turning Actuator", 5));
        actuatorList.add(new ActuatorConfig("Back Left Turning Actuator", 6));
        actuatorList.add(new ActuatorConfig("Back Right Turning Actuator", 7));
        actuatorList.add(new ActuatorConfig("Bucket Conveyor Drive Motor", 8));
        actuatorList.add(new ActuatorConfig("Bucket Conveyor Linear Motor", 9));
        actuatorList.add(new ActuatorConfig("Bucket Conveyor Actuators", 10));
        actuatorList.add(new ActuatorConfig("Deposition Motor", 11));
        actuatorList.add(new ActuatorConfig("Deposition Actuators", 12));

         // Add sensors
        for (ActuatorConfig config : actuatorList){
            hci.addActuator(config);
        }
    }

    private static void routeLocomotionMessage(String[] keys, byte[] body){
        if(keys.length < 4) {
            System.out.println("Locomotion motor control routing key must have 4 elements");
            return;
        }

        if (keys[3].equals("wheel_rpm")) {
            routeWheelRPMMessage(keys, body);

        } else if (keys[3].equals("wheel_pod_pos")) {
            routeWheelPodPosMessage(keys, body);

        } else {
            System.out.println("Locomotion motor control routing key has unrecognized motor");
            return;
        }
    }

    private static void routeWheelRPMMessage(String[] keys, byte[] body){
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
        Messages.SpeedContolCommand scc = Messages.SpeedContolCommand.parseFrom(body);
        if (id % 2 == 0) { //Left wheels need reversed directions
            targetValue = -Mechanics.wheelRPMToValue(scc.getRpm());
        } else {
            targetValue = Mechanics.wheelRPMToValue(scc.getRpm());
        }
        queueActuation(id, targetValue);
    }

    private static void routeWheelPodPosMessage(String[] keys, byte[] body){
        int id = -1;
        if (keys[2].equals("front_left")) {
            id = 4;
        } else if (keys[2].equals("front_right")) {
            id = 5;
        } else if (keys[2].equals("back_left")) {
            id = 6;
        } else if (keys[2].equals("back_right")) {
            id = 7;
        } else {
            System.out.println("Locomotion motor control routing key has invalid wheel");
            return;
        }
        Messages.PositionContolCommand pcc = Messages.PositionContolCommand.parseFrom(body);
        double targetValue = Mechanics.wheelPodPosToValue(pcc.getPosition());
        queueActuation(id, targetValue);
    }

    private static void routeExcavationMessage(String[] keys, byte[] body){
        if(keys.length < 3) {
            System.out.println("Excavation motor control routing key must have 3 elements");
            return;
        }

        if (keys[2].equals("conveyor_translation_displacement")) {
            Messages.PositionContolCommand pcc = Messages.PositionContolCommand.parseFrom(body);
            int id = 9;
            double targetValue = ((pcc.getPosition() / 100.0F) * 785) + 45;
            queueActuation(id, targetValue);
        } else if (keys[2].equals("arm_pos")) {
            Messages.PositionContolCommand pcc = Messages.PositionContolCommand.parseFrom(body);
            int id = 10;
            double targetValue = ((90- pcc.getPosition()) * 5.555) + 100 ;
            queueActuation(id, targetValue);
        } else if (keys[2].equals("bucket_conveyor_rpm")) {
            Messages.SpeedContolCommand scc = Messages.SpeedContolCommand.parseFrom(body);
            int id = 8;
            double targetValue = (scc.getRpm() / 100.0F) * 32767;
            queueActuation(id, targetValue);
        } else {
            System.out.println("Excavation motor control routing key has unrecognized motor");
            return;
        }
    }

    private static void routeDepositionMessage(String[] keys, byte[] body){
        if(keys.length < 3) {
            System.out.println("Deposition motor control routing key must have 3 elements");
            return;
        }
        if (keys[2].equals("dump_pos")) {
            Messages.PositionContolCommand pcc = Messages.PositionContolCommand.parseFrom(body);
            int id = 12;
            double targetValue = (pcc.getPosition() / 100.0F) * 127;
            queueActuation(id, targetValue);
        } else if (keys[2].equals("conveyor_rpm")) {
            Messages.SpeedContolCommand scc = Messages.SpeedContolCommand.parseFrom(body);
            int id = 11;
            double targetValue = (scc.getRpm() / 100.0F) * 127;
            queueActuation(id, targetValue);
        } else if (keys[2].equals("vibration_rpm")) {
            Messages.SpeedContolCommand scc = Messages.SpeedContolCommand.parseFrom(body);
            int id = 13;
            double targetValue = (scc.getRpm() / 100.0F) * 255;
            queueActuation(id, targetValue);
        } else {
            System.out.println("Deposition motor control routing key has unrecognized motor");
            return;
        }
    }

    private static void routeSystemMessage(String[] keys, byte[] body){
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
