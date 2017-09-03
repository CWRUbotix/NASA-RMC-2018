import pika
import messages_pb2

queue_name = 'locomotion'

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
    msg_in = messages_pb2.LocomotionControl()
    msg_in.ParseFromString(body)
    print(msg_in)
    msg_out = messages_pb2.MultiServoControl()
    front_left_control = msg_out.element.add()
    front_right_control = msg_out.element.add()
    back_left_control = msg_out.element.add()
    back_right_control = msg_out.element.add()
    front_left_control.motorID = 0
    front_left_control.timeout_ms = msg_in.timeout_ms
    front_right_control.motorID = 1
    front_right_control.timeout_ms = msg_in.timeout_ms
    back_left_control.motorID = 2
    back_left_control.timeout_ms = msg_in.timeout_ms
    back_right_control.motorID = 3
    back_right_control.timeout_ms = msg_in.timeout_ms

    speed_multipliers = (0,0,0,0)
    if (msg_in.locomotionType is messages_pb2.LocomotionControl.FORWARD):
        speed_multipliers = (1,1,1,1)
    elif (msg_in.locomotionType is messages_pb2.LocomotionControl.BACKWARD):
        speed_multipliers = (-1,-1,-1,-1)
    elif (msg_in.locomotionType is messages_pb2.LocomotionControl.LEFT):
        speed_multipliers = (-1,1,-1,1)
    elif (msg_in.locomotionType is messages_pb2.LocomotionControl.RIGHT):
        speed_multipliers = (1,-1,1,-1)
    
    front_left_control.speed_percent = msg_in.speed_percent * speed_multipliers[0]
    front_right_control.speed_percent = msg_in.speed_percent * speed_multipliers[1]
    back_left_control.speed_percent = msg_in.speed_percent * speed_multipliers[2]
    back_right_control.speed_percent = msg_in.speed_percent * speed_multipliers[3]
    channel.basic_publish('', 'servo_control', msg_out.SerializeToString())
    print('sent')
    print(msg_out)
    
    

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
