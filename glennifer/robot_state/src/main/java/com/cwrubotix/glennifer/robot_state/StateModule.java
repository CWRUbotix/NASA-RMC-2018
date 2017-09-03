package com.cwrubotix.glennifer.robot_state;


import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.DefaultConsumer;
import com.rabbitmq.client.Envelope;

import com.rabbitmq.client.AMQP;

import com.cwrubotix.glennifer.Messages;
import com.cwrubotix.glennifer.Messages.LoadUpdate;
import com.cwrubotix.glennifer.Messages.RpmUpdate;
import com.cwrubotix.glennifer.Messages.LimitUpdate;
import com.cwrubotix.glennifer.Messages.PositionUpdate;
import com.cwrubotix.glennifer.Messages.DisplacementUpdate;
import com.cwrubotix.glennifer.Messages.CurrentUpdate;
import com.cwrubotix.glennifer.Messages.Fault;
import com.cwrubotix.glennifer.Messages.UnixTime;


import java.io.IOException;
import java.time.Instant;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.TimeoutException;

/**
 *
 * @author Michael
 */
public class StateModule {

    class SubscriptionRunnable implements Runnable {

        private String returnKey;
        private int interval_ms;
        private boolean loc_summary;
        private boolean loc_detailed;
        private boolean exc_summary;
        private boolean exc_detailed;
        private boolean dep_summary;
        private boolean dep_detailed;

        public SubscriptionRunnable(String returnKey, int interval_ms, boolean loc_summary, boolean loc_detailed,
                                                                        boolean exc_summary, boolean exc_detailed,
                                                                        boolean dep_summary, boolean dep_detailed) {
            this.returnKey = returnKey;
            this.interval_ms = interval_ms;
            this.loc_summary = loc_summary;
            this.loc_detailed = loc_detailed;
            this.exc_summary = exc_summary;
            this.exc_detailed = exc_detailed;
            this.dep_summary = dep_summary;
            this.dep_detailed = dep_detailed;
        }

        @Override
        public void run() {
            boolean go = true;
            while (go) {
                Instant now = Instant.now();
                Messages.State.Builder stateMsgBuilder = Messages.State.newBuilder();
                stateMsgBuilder.setTimestamp(instantToUnixTime(now));
                if (loc_summary) {
                    Messages.LocomotionStateSummary msg = Messages.LocomotionStateSummary.newBuilder()
                            .setConfig(Messages.LocomotionStateSummary.Configuration.valueOf(StateModule.this.locomotionState.getConfiguration().ordinal()))
                            .setSpeed(StateModule.this.locomotionState.getStraightSpeed())
                            .build();
                    stateMsgBuilder.setLocSummary(msg);
                }
                if (loc_detailed) {
                    Messages.LocomotionStateDetailed msg = Messages.LocomotionStateDetailed.newBuilder()
                            .setFrontLeftRpm(locomotionState.getWheelRpm(LocomotionState.Wheel.FRONT_LEFT))
                            .setFrontRightRpm(locomotionState.getWheelRpm(LocomotionState.Wheel.FRONT_RIGHT))
                            .setBackLeftRpm(locomotionState.getWheelRpm(LocomotionState.Wheel.BACK_LEFT))
                            .setBackRightRpm(locomotionState.getWheelRpm(LocomotionState.Wheel.BACK_RIGHT))
                            .setFrontLeftPos(locomotionState.getWheelPodPos(LocomotionState.Wheel.FRONT_LEFT))
                            .setFrontRightPos(locomotionState.getWheelPodPos(LocomotionState.Wheel.FRONT_RIGHT))
                            .setBackLeftPos(locomotionState.getWheelPodPos(LocomotionState.Wheel.BACK_LEFT))
                            .setBackRightPos(locomotionState.getWheelPodPos(LocomotionState.Wheel.BACK_RIGHT))
                            .setFrontLeftExtended(locomotionState.getWheelPodLimitExtended(LocomotionState.Wheel.FRONT_LEFT))
                            .setFrontRightExtended(locomotionState.getWheelPodLimitExtended(LocomotionState.Wheel.FRONT_RIGHT))
                            .setBackLeftExtended(locomotionState.getWheelPodLimitExtended(LocomotionState.Wheel.BACK_LEFT))
                            .setBackRightExtended(locomotionState.getWheelPodLimitExtended(LocomotionState.Wheel.BACK_RIGHT))
                            .setFrontLeftRetracted(locomotionState.getWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_LEFT))
                            .setFrontRightRetracted(locomotionState.getWheelPodLimitRetracted(LocomotionState.Wheel.FRONT_RIGHT))
                            .setBackLeftRetracted(locomotionState.getWheelPodLimitRetracted(LocomotionState.Wheel.BACK_LEFT))
                            .setBackRightRetracted(locomotionState.getWheelPodLimitRetracted(LocomotionState.Wheel.BACK_RIGHT))
                            .build();
                    stateMsgBuilder.setLocDetailed(msg);
                }
                if (exc_summary) {
                    Messages.ExcavationStateSummary msg = Messages.ExcavationStateSummary.newBuilder()
                            .setRpm(excavationState.getConveyorRpm())
                            .setArmPos(excavationState.getArmPos())
                            .setDisplacement(excavationState.getTranslationDisplacement())
                            .setArmExtended(excavationState.getArmExtended())
                            .setArmRetracted(excavationState.getArmRetracted())
                            .setTranslationExtended(excavationState.getTranslationExtended())
                            .setTranslationRetracted(excavationState.getTranslationRetracted())
                            .build();
                    stateMsgBuilder.setExcSummary(msg);
                }
                if (exc_detailed) {
                    Messages.ExcavationStateDetailed msg = Messages.ExcavationStateDetailed.newBuilder()
                            .setRpm(excavationState.getConveyorRpm())
                            .setArmPos(excavationState.getArmPos())
                            .setDisplacement(excavationState.getTranslationDisplacement())
                            .setArmLeftExtended(excavationState.getArmExtended(ExcavationState.Side.LEFT))
                            .setArmRightExtended(excavationState.getArmExtended(ExcavationState.Side.RIGHT))
                            .setArmLeftRetracted(excavationState.getArmRetracted(ExcavationState.Side.LEFT))
                            .setArmRightRetracted(excavationState.getArmRetracted(ExcavationState.Side.RIGHT))
                            .setTranslationLeftExtended(excavationState.getTranslationExtended(ExcavationState.Side.LEFT))
                            .setTranslationRightExtended(excavationState.getTranslationExtended(ExcavationState.Side.RIGHT))
                            .setTranslationLeftRetracted(excavationState.getTranslationRetracted(ExcavationState.Side.LEFT))
                            .setTranslationRightRetracted(excavationState.getTranslationRetracted(ExcavationState.Side.RIGHT))
                            .setConveyorMotorCurrent(excavationState.getConveyorMotorCurrent())
                            .build();
                    stateMsgBuilder.setExcDetailed(msg);
                }
                if (dep_summary) {
                    Messages.DepositionStateSummary msg = Messages.DepositionStateSummary.newBuilder()
                            .setPos(depositionState.getDumpPos())
                            .setLoad(depositionState.getDumpLoad())
                            .setDumpExtended(depositionState.getDumpExtended())
                            .setDumpRetracted(depositionState.getDumpRetracted())
                            .build();
                    stateMsgBuilder.setDepSummary(msg);
                }
                if (dep_detailed) {
                    Messages.DepositionStateDetailed msg = Messages.DepositionStateDetailed.newBuilder()
                            .setFrontLeftLoad(depositionState.getDumpLoad(DepositionState.LoadCell.FRONT_LEFT))
                            .setFrontRightLoad(depositionState.getDumpLoad(DepositionState.LoadCell.FRONT_RIGHT))
                            .setBackLeftLoad(depositionState.getDumpLoad(DepositionState.LoadCell.BACK_LEFT))
                            .setBackRightLoad(depositionState.getDumpLoad(DepositionState.LoadCell.BACK_RIGHT))
                            .setDumpLeftExtended(depositionState.getDumpExtended(DepositionState.Side.LEFT))
                            .setDumpRightExtended(depositionState.getDumpExtended(DepositionState.Side.RIGHT))
                            .setDumpLeftRetracted(depositionState.getDumpRetracted(DepositionState.Side.LEFT))
                            .setDumpRightRetracted(depositionState.getDumpRetracted(DepositionState.Side.RIGHT))
                            .build();
                    stateMsgBuilder.setDepDetailed(msg);
                }
                try {
                    //Not sure how to do this bit properly
                    StateModule.this.channel.basicPublish(exchangeName, returnKey, null, stateMsgBuilder.build().toByteArray());
                    Thread.sleep(interval_ms);
                } catch (IOException e) {
                    go = false;
                    e.printStackTrace();
                } catch (InterruptedException e) {
                    go = false;
                }
            }
        }
    }

    /* Consumer callback class and methods */
    private class UpdateConsumer extends DefaultConsumer {
        
        public UpdateConsumer(Channel channel) {
            super(channel);
        }

        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            String routingKey = envelope.getRoutingKey();
            String[] keys = routingKey.split("\\.");
            if (keys.length < 2) {
                return;
            }
            String typeOfSensor = keys[1];

            if(typeOfSensor.equals("locomotion")){ //this is a locomotion message
                if (keys.length < 4) {
                    System.out.println("Locomotion sensor update routing key requires 4 elements");
                    return;
                }
                String wheelString = keys[2];
                String sensorString = keys[3];
                LocomotionState.Wheel wheel;
                if (wheelString.equals("front_left")) {
                    wheel = LocomotionState.Wheel.FRONT_LEFT;
                } else if (wheelString.equals("front_right")) {
                    wheel = LocomotionState.Wheel.FRONT_RIGHT;
                } else if (wheelString.equals("back_left")) {
                    wheel = LocomotionState.Wheel.BACK_LEFT;
                } else if (wheelString.equals("back_right")) {
                    wheel = LocomotionState.Wheel.BACK_RIGHT;
                } else {
                    System.out.println("Bad wheel string in routing key");
                    return;
                }
                if (sensorString.equals("wheel_rpm")) {
                    handleWheelRpmUpdate(wheel, body);
                } else if (sensorString.equals("wheel_pod_pos")) {
                    handleWheelPodPositionUpdate(wheel, body);
                } else if (sensorString.equals("wheel_pod_limit_extended")) {
                    handleWheelPodLimitExtendedUpdate(wheel, body);
                } else if (sensorString.equals("wheel_pod_limit_retracted")) {
                    handleWheelPodLimitRetractedUpdate(wheel, body);
                } else {
                    System.out.println("Bad sensor string in routing key");
                    return;
                }
            }
            else if(typeOfSensor.equals("excavation")){ //excavation message
                if (keys.length < 3) {
                    System.out.println("Excavation sensor update routing key requires 3 elements");
                    return;
                }
                String sensorString = keys[2];

                if (sensorString.equals("conveyor_rpm")) {
                    handleConveyorRpmUpdate(body);
                } else if (sensorString.equals("translation_pos")) {
                    handleConveyorTranslationPosUpdate(body);
                }  else if (sensorString.equals("arm_pos_a")) {
                    handleArmPosUpdate(body);
                }  else if (sensorString.equals("arm_pos_b")) {
                    // do nothing for now
                } else if (sensorString.equals("arm_limit_extended")) {
                    String sideString = keys[3];
                    ExcavationState.Side side;
                    if (sideString.equals("left")) {
                        side = ExcavationState.Side.LEFT;
                    } else if (sideString.equals("right")) {
                        side = ExcavationState.Side.RIGHT;
                    } else {
                        System.out.println("Bad side string in routing key");
                        return;
                    }
                    handleArmLimitExtendedUpdate(side, body);
                } else if (sensorString.equals("arm_limit_retracted")) {
                    String sideString = keys[3];
                    ExcavationState.Side side;
                    if (sideString.equals("left")) {
                        side = ExcavationState.Side.LEFT;
                    } else if (sideString.equals("right")) {
                        side = ExcavationState.Side.RIGHT;
                    } else {
                        System.out.println("Bad side string in routing key");
                        return;
                    }
                    handleArmLimitRetractedUpdate(side, body);
                } else if (sensorString.equals("conveyor_translation_limit_extended")) {
                    String sideString = keys[3];
                    ExcavationState.Side side;
                    if (sideString.equals("left")) {
                        side = ExcavationState.Side.LEFT;
                    } else if (sideString.equals("right")) {
                        side = ExcavationState.Side.RIGHT;
                    } else {
                        System.out.println("Bad side string in routing key");
                        return;
                    }
                    handleConveyorTranslationLimitExtendedUpdate(side, body);
                } else if (sensorString.equals("conveyor_translation_limit_retracted")) {
                    String sideString = keys[3];
                    ExcavationState.Side side;
                    if (sideString.equals("left")) {
                        side = ExcavationState.Side.LEFT;
                    } else if (sideString.equals("right")) {
                        side = ExcavationState.Side.RIGHT;
                    } else {
                        System.out.println("Bad side string in routing key");
                        return;
                    }
                    handleConveyorTranslationLimitRetractedUpdate(side, body);
                } else if (sensorString.equals("arm_pos_a")) { // TODO: both sides
                    handleConveyorTranslationPosUpdate(body);
                } else if (sensorString.equals("conveyor_current")){
                    handleConveyorMotorCurrentUpdate(body);
                } else {
                    System.out.println("Bad sensor string in routing key: " + sensorString);
                    return;
                }
            }
            else if(typeOfSensor.equals("deposition")){ //deposition message
                String sensorString = keys[2];
                if (sensorString.equals("dump_load")) {
                    String loadCellString = keys[3];
                    DepositionState.LoadCell loadcell;
                    if(loadCellString.equals("front_left")){
                        loadcell = DepositionState.LoadCell.FRONT_LEFT;
                    } else if (loadCellString.equals("front_right")){
                        loadcell = DepositionState.LoadCell.FRONT_RIGHT;
                    } else if (loadCellString.equals("back_left")){
                        loadcell = DepositionState.LoadCell.BACK_LEFT;
                    } else if (loadCellString.equals("back_right")) {
                        loadcell = DepositionState.LoadCell.BACK_RIGHT;
                    } else {
                        System.out.println("Bad load cell string in routing key");
                        return;
                    }
                    handleDumpLoadUpdate(loadcell, body);
                } else if (sensorString.equals("arm_pos")) {
                    handleDumpPosUpdate(body);
                } else if (sensorString.equals("dump_limit_extended")) {
                    String sideString = keys[3];
                    DepositionState.Side side;
                    if (sideString.equals("left")) {
                        side = DepositionState.Side.LEFT;
                    } else if (sideString.equals("right")) {
                        side = DepositionState.Side.RIGHT;
                    } else {
                        System.out.println("Bad side string in routing key");
                        return;
                    }
                    handleDumpLimitExtendedUpdate(side, body);
                } else if (sensorString.equals("dump_limit_retracted")) {
                    String sideString = keys[3];
                    DepositionState.Side side;
                    if (sideString.equals("left")) {
                        side = DepositionState.Side.LEFT;
                    } else if (sideString.equals("right")) {
                        side = DepositionState.Side.RIGHT;
                    } else {
                        System.out.println("Bad side string in routing key");
                        return;
                    }
                    handleDumpLimitRetractedUpdate(side, body);
                } else {
                    System.out.println("Bad sensor string in routing key");
                }
            } else { //oops
                System.out.println("Bad subsystem string in routing key");
                return;
            }

        }
    }

    private class RequestConsumer extends DefaultConsumer {

        public RequestConsumer(Channel channel) {
            super(channel);
        }

        @Override
        public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
            Messages.StateSubscribe msg = Messages.StateSubscribe.parseFrom(body);
            float interval = msg.getInterval();
            int interval_ms = (int)(interval * 1000);
            boolean loc_summary = true;
            boolean loc_detailed = true;
            boolean exc_summary = true;
            boolean exc_detailed = true;
            boolean dep_summary = true;
            boolean dep_detailed = true;
            String replyKey = msg.getReplyKey();
            Thread t = new Thread(new SubscriptionRunnable(
                    replyKey,
                    interval_ms,
                    loc_summary,
                    loc_detailed,
                    exc_summary,
                    exc_detailed,
                    dep_summary,
                    dep_detailed), replyKey);
            subscriptionThreads.add(t, -1);
            t.start();
            System.out.println("Start subscription thread with interval = " + interval);
        }
    }
    
    private class UnsubscriptionRequest extends DefaultConsumer{
    	
    	public UnsubscriptionRequest(Channel channel){
    		super(channel);
    	}
    	
    	@Override
    	public void handleDelivery(String consumerTag, Envelope envelope, AMQP.BasicProperties properties, byte[] body) throws IOException {
    		Messages.StateSubscribe msg = Messages.StateSubscribe.parseFrom(body);
    		String replyKey = msg.getReplyKey();
    		int index = subscriptionThreads.indexOf(replyKey, -1);
            try{
    		  Thread t = subscriptionThreads.get(index);
    		  subscriptionThreads.remove(t);
    		  t.interrupt();
        		try {
	   			t.join();
		      	} catch (InterruptedException e) {
			     	// TODO Auto-generated catch block
			     	e.printStackTrace();
                 }
			} catch (ArrayIndexOutOfBoundsException e1) {
                System.out.println("The Thread you are trying to terminate does not exist.");
            }
    	}
    }
    
    private void handleWheelRpmUpdate(LocomotionState.Wheel wheel, byte[] body) throws IOException {
        RpmUpdate message = RpmUpdate.parseFrom(body);
        float rpm = message.getRpm();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            locomotionState.updateWheelRpm(wheel, rpm, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }
    
    private void handleWheelPodPositionUpdate(LocomotionState.Wheel wheel, byte[] body) throws IOException {
        PositionUpdate message = PositionUpdate.parseFrom(body);
        float pos = message.getPosition();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            locomotionState.updateWheelPodPos(wheel, pos, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }
        
    private void handleWheelPodLimitExtendedUpdate(LocomotionState.Wheel wheel, byte[] body) throws IOException {
        LimitUpdate message = LimitUpdate.parseFrom(body);
        boolean pressed = message.getPressed();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            locomotionState.updateWheelPodLimitExtended(wheel, pressed, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }
            
    private void handleWheelPodLimitRetractedUpdate(LocomotionState.Wheel wheel, byte[] body) throws IOException {
        LimitUpdate message = LimitUpdate.parseFrom(body);
        boolean pressed = message.getPressed();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            locomotionState.updateWheelPodLimitRetracted(wheel, pressed, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleConveyorRpmUpdate(byte[] body) throws IOException {
        RpmUpdate message = RpmUpdate.parseFrom(body);
        float rpm = message.getRpm();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            excavationState.updateConveyorRpm(rpm, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleConveyorMotorCurrentUpdate(byte[] body) throws IOException {
        CurrentUpdate message = CurrentUpdate.parseFrom(body);
        float current = message.getCurrent();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            excavationState.updateConveyorMotorCurrent(current, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleConveyorTranslationPosUpdate(byte[] body) throws IOException {
        PositionUpdate message = PositionUpdate.parseFrom(body);
        float pos = message.getPosition();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            excavationState.updateTranslationDisplacement(pos, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleArmPosUpdate(byte[] body) throws IOException {
        PositionUpdate message = PositionUpdate.parseFrom(body);
        float pos = message.getPosition();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            excavationState.updateArmPos(pos, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleArmLimitExtendedUpdate(ExcavationState.Side side, byte[] body) throws IOException {
        LimitUpdate message = LimitUpdate.parseFrom(body);
        boolean pressed = message.getPressed();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            excavationState.updateArmLimitExtended(side, pressed, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleArmLimitRetractedUpdate(ExcavationState.Side side, byte[] body) throws IOException {
        LimitUpdate message = LimitUpdate.parseFrom(body);
        boolean pressed = message.getPressed();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            excavationState.updateArmLimitRetracted(side, pressed, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleConveyorTranslationLimitExtendedUpdate(ExcavationState.Side side, byte[] body) throws IOException {
        LimitUpdate message = LimitUpdate.parseFrom(body);
        boolean pressed = message.getPressed();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            excavationState.updateTranslationLimitExtended(side, pressed, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleConveyorTranslationLimitRetractedUpdate(ExcavationState.Side side, byte[] body) throws IOException {
        LimitUpdate message = LimitUpdate.parseFrom(body);
        boolean pressed = message.getPressed();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            excavationState.updateTranslationLimitRetracted(side, pressed, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleDumpLoadUpdate(DepositionState.LoadCell cell, byte[] body) throws IOException {
        LoadUpdate message = LoadUpdate.parseFrom(body);
        float load = message.getLoad();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            depositionState.updateDumpLoad(cell, load, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleDumpPosUpdate(byte[] body) throws IOException {
        PositionUpdate message = PositionUpdate.parseFrom(body);
        float pos = message.getPosition();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            depositionState.updateDumpPos(pos, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleDumpLimitExtendedUpdate(DepositionState.Side side, byte[] body) throws IOException {
        LimitUpdate message = LimitUpdate.parseFrom(body);
        boolean pressed = message.getPressed();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            depositionState.updateDumpLimitExtended(side, pressed, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }

    private void handleDumpLimitRetractedUpdate(DepositionState.Side side, byte[] body) throws IOException {
        LimitUpdate message = LimitUpdate.parseFrom(body);
        boolean pressed = message.getPressed();
        Instant time = Instant.ofEpochSecond(message.getTimestamp().getTimeInt(), (long)(message.getTimestamp().getTimeFrac() * 1000000000L));
        try {
            depositionState.updateDumpLimitRetracted(side, pressed, time);
        } catch (RobotFaultException e) {
            sendFault(e.getFaultCode(), time);
        }
    }
    
    /*Hash Table for SubscriptionThreads*/
    private class SubscriptionThreads{

    	private Thread[] threads;
    	private int numItems;
    	
    	public SubscriptionThreads(){
    		threads = new Thread[4096];
    		numItems = 0;
    	}
    	
    	//Generates hashcode with given replyKey of the thread
    	public int hashCode(String replyKey){
    		int hashCode = 0;
    		int power = 0;
    		for(int index = 0; index < replyKey.length(); index++){
    			hashCode = hashCode + ((replyKey.charAt(index) << power) - replyKey.charAt(index));
    		}
    		return hashCode % threads.length;
    	}
    	
    	public synchronized void add(Thread t, int currentIndex){
    		if(currentIndex == -1)
    			currentIndex = hashCode(t.getName());
    		if(threads[currentIndex] == null || threads[currentIndex].getName().equals("removed")){
    			threads[currentIndex] = t;
    			numItems++;
    			if(((double)numItems/(double)threads.length) > 0.75)
    				rehash();
    		}
    		else if(!threads[currentIndex].getName().equals(t.getName())){
    			currentIndex = (currentIndex + 7) % threads.length;
    			add(t, currentIndex);
    		} 
    		else
    			return;
    	}
    	
    	public synchronized Thread get(int index){
    		return threads[index];
    	}
    	
    	public synchronized int indexOf(String replyKey, int currentIndex){
    		if(currentIndex == -1)
    			currentIndex = hashCode(replyKey);
    		if(threads[currentIndex] == null)
    			return -1; 
    		else if(!threads[currentIndex].getName().equals(replyKey)){
    			currentIndex = (currentIndex + 7) % threads.length; // second hash function with stride of 7.
    			return indexOf(replyKey, currentIndex);
    		} 
    		else
    			return currentIndex;
    	}
    	
    	public synchronized void remove(Thread t){
    		int index = indexOf(t.getName(), -1);
    		threads[index] = new Thread("removed");
    	}
    	
    	public synchronized void rehash(){
    		Thread[] copy = threads.clone();
    		threads = new Thread[copy.length*2];
    		for(int index = 0; index < copy.length; index++){
    			if(copy[index] != null && !copy[index].getName().equals("removed"))
    				add(copy[index], hashCode(copy[index].getName())); 
    		}
    	}
    }
    
    /* Data Members */
    private LocomotionState locomotionState;
    private ExcavationState excavationState;
    private DepositionState depositionState;
    private String exchangeName;
    private Connection connection;
    private Channel channel;
    private SubscriptionThreads subscriptionThreads = new SubscriptionThreads();
    
    public StateModule(LocomotionState locState, ExcavationState excState, DepositionState depState) {
        this(locState, excState, depState, "amq.topic");
    }

    public StateModule(LocomotionState locState, ExcavationState excState, DepositionState depState, String exchangeName) {
        this.locomotionState = locState;
        this.excavationState = excState;
        this.depositionState = depState;
        this.exchangeName = exchangeName;
    }
    
    private UnixTime instantToUnixTime(Instant time) {
        UnixTime.Builder unixTimeBuilder = UnixTime.newBuilder();
        unixTimeBuilder.setTimeInt(time.getEpochSecond());
        unixTimeBuilder.setTimeFrac(time.getNano() / 1000000000F);
        return unixTimeBuilder.build();
    }
    
    private void sendFault(int faultCode, Instant time) throws IOException {
        Fault.Builder faultBuilder = Fault.newBuilder();
        faultBuilder.setFaultCode(faultCode);
        faultBuilder.setTimestamp(instantToUnixTime(time));
        Fault message = faultBuilder.build();
        channel.basicPublish(exchangeName, "fault", null, message.toByteArray());
    }
    
    public void runWithExceptions() throws IOException, TimeoutException {
        // Setup connection
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        connection = factory.newConnection();
        this.channel = connection.createChannel();

        // Subscribe to sensor updates
        String queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "sensor.#");
        this.channel.basicConsume(queueName, true, new UpdateConsumer(channel));

        // Listen for requests to subscribe to state updates
        queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "state.subscribe");
        this.channel.basicConsume(queueName, true, new RequestConsumer(channel));
        
        // Listen for requests to unsubscribe to state updates.
        queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, exchangeName, "state.unsubscribe");
        this.channel.basicConsume(queueName, true, new UnsubscriptionRequest(channel));
    }

    public void start() {
        try {
            runWithExceptions();
        } catch (Exception e) {
            try {
                sendFault(999, Instant.now());
            } catch (Exception e2) { }
            e.printStackTrace();
            System.out.println(e.getMessage());
        }
    }

    public void stop() throws IOException, TimeoutException, InterruptedException {
        for (Thread t : subscriptionThreads.threads) {
            t.interrupt();
        }
        for (Thread t : subscriptionThreads.threads) {
            t.join();
        }
        channel.close();
        connection.close();
    }
    
    public static void main(String[] args) {
        LocomotionState locState = new LocomotionState();
        ExcavationState excState = new ExcavationState();
        DepositionState depState = new DepositionState();
        StateModule module = new StateModule(locState, excState, depState);
        module.start();
    }
}