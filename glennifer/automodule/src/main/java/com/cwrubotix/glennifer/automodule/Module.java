package com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;

import com.cwrubotix.glennifer.Messages.UnixTime;
import com.cwrubotix.glennifer.Messages.Fault;

import java.io.IOException;
import java.time.Instant;
import java.util.concurrent.TimeoutException;

public abstract class Module {
    protected String exchangeName;
    protected Connection connection;
    protected Channel channel;

    //amqp stuff
    protected UnixTime instantToUnixTime(Instant time) {
        UnixTime.Builder unixTimeBuilder = UnixTime.newBuilder();
        unixTimeBuilder.setTimeInt(time.getEpochSecond());
        unixTimeBuilder.setTimeFrac(time.getNano() / 1000000000F);
        return unixTimeBuilder.build();
    }

    //amqp stuff
    protected void sendFault(int faultCode, Instant time) throws IOException {
        Fault.Builder faultBuilder = Fault.newBuilder();
        faultBuilder.setFaultCode(faultCode);
        faultBuilder.setTimestamp(instantToUnixTime(time));
        Fault message = faultBuilder.build();
        channel.basicPublish(exchangeName, "fault", null, message.toByteArray());
    }

    abstract protected void runWithExceptions() throws IOException, TimeoutException;

    public void start(){
        try{
            runWithExceptions();
        } catch(Exception e){
            try{
                sendFault(999, Instant.now());
            } catch(Exception e1){
                // Do nothing
            }
            e.printStackTrace();
            System.out.println(e.getMessage());
        }
    }

    public void stop() {
        try {
            channel.close();
            connection.close();
        } catch (IOException | TimeoutException e) {
            // Do nothing
        }
    }

}
