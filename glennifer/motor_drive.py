import pika
import messages_pb2
import serial
import threading

queue_name = 'motor_drive'

ser_connected = True

try:
    ser = serial.Serial('COM5', 9600, timeout=1)
except:
    print("Warning: serial not connected")
    ser_connected = False

def timeout_callback(motorID):
    if ser_connected:
        ser.write(bytes('M' + str(motorID+1) + ': ' + str(0) + '\n', 'ascii'))
    print('set ' + str(motorID) + ' to ' + str(0))

motor_events = {0:None,
                1:None,
                2:None,
                3:None}

# Create a global channel variable to hold our channel object in
channel = None

# Step #2
def on_connected(connection):
    """Called when we are fully connected to RabbitMQ"""
    # Open a channel
    connection.channel(on_channel_open)

# Step #3
def on_channel_open(new_channel):
    """Called when our channel has opened"""
    global channel
    channel = new_channel
    channel.queue_declare(queue=queue_name, durable=True, exclusive=False, auto_delete=False, callback=on_queue_declared)

# Step #4
def on_queue_declared(frame):
    """Called when RabbitMQ has told us our Queue has been declared, frame is the response from RabbitMQ"""
    channel.basic_consume(handle_delivery, queue=queue_name, no_ack = True)

# Step #5
def handle_delivery(channel, method, header, body):
    """Called when we receive a message from RabbitMQ"""
    print('recieved')
    msg_in = messages_pb2.MultiMotorDrive()
    msg_in.ParseFromString(body)
    print(msg_in)
    for smc in msg_in.element:
        if ser_connected:
            voltage_percent = smc.voltage_percent
            if (smc.motorID == 1):
                voltage_percent *= -1
            ser.write(bytes('M' + str(smc.motorID+1) + ': ' + str(20*voltage_percent) + '\n', 'ascii'))
        print('set ' + str(smc.motorID) + ' to ' + str(smc.voltage_percent))

        # cancel old timeout event
        if motor_events[smc.motorID]:
            motor_events[smc.motorID].cancel()

        # create and store new timeout event
        motor_event = threading.Timer(smc.timeout_ms / 100.0,
                                      timeout_callback,
                                      kwargs={
                                          'motorID': smc.motorID})
        motor_event.start()
        motor_events[smc.motorID] = motor_event
    
    

# Step #1: Connect to RabbitMQ using the default parameters
parameters = pika.connection.URLParameters('amqp://guest:guest@localhost:5672/%2F')
connection = pika.SelectConnection(parameters=parameters,
                                   on_open_callback=on_connected)
try:
    # Loop so we can communicate with RabbitMQ
    connection.ioloop.start()
except KeyboardInterrupt:
    # Gracefully close the connection
    connection.close()
    # Loop until we're fully closed, will stop on its own
    connection.ioloop.start()
