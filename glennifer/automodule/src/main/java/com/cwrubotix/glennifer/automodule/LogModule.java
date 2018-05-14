package com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.ConnectionFactory;

import java.io.*;
import java.time.LocalDateTime;
import java.util.concurrent.TimeoutException;

/**
 * Module to collect data and write it to a file
 * @author Imran Hossain
 * @author Seohyun Jung
 */
public class LogModule extends Module {
    /// Robot-specific fields
    PathFinder pathFinder;

    /// Logging fields
    private File pathLogFile, driveLogFile;
    private Writer pathLogWriter, driveLogWriter;

    public LogModule() {
        this("amq.topic");
    }

    public LogModule(String exchangeName) {
        this.exchangeName = exchangeName;
    }

    @Override
    protected void runWithExceptions() throws IOException, TimeoutException {
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        this.connection = factory.newConnection();
        this.channel = connection.createChannel();

        // Create log files and writers
        LocalDateTime openTime = LocalDateTime.now();
        pathLogFile = new File(String.format("%d-%d-%d-%d:%d_Path.txt",
                openTime.getYear(), openTime.getMonthValue(), openTime.getDayOfMonth(),
                openTime.getHour(), openTime.getMinute()));
        pathLogWriter = new BufferedWriter(new FileWriter(pathLogFile));
        driveLogFile = new File(String.format("%d-%d-%d-%d:%d_Drive.txt",
                openTime.getYear(), openTime.getMonthValue(), openTime.getDayOfMonth(),
                openTime.getHour(), openTime.getMinute()));
        driveLogWriter = new BufferedWriter(new FileWriter(driveLogFile));
    }

    public static void main(String[] args) {
        LogModule logModule = new LogModule();
        logModule.start();
    }
}
