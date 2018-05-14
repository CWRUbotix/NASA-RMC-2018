package com.cwrubotix.glennifer.automodule;

import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.TimeoutException;

/**
 * Module to collect data and write it to a file
 */
public class LogModule extends Module {
    /// Messaging Fields
    public String exchangeName;
    private Connection connection;
    private Channel channel;

    /// Robot-specific fields
    PathFinder pathFinder;

    /// Logging fields
    File logFile;
    BufferedWriter logWriter;

    @Override
    protected void runWithExceptions() throws IOException, TimeoutException {

    }
}
