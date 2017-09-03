import pika
import messages_pb2
from enum import Enum

class Wheel(Enum):
    front_left = 1
    front_right = 2
    back_left = 3
    back_right = 4

# Declare 4 queue names and corresponding 4 binding keys
straight_queue = 'subsyscommand.locomotion.straight'
turn_queue = 'subsyscommand.locomotion.turn'
strafe_queue = 'subsyscommand.locomotion.strafe'
configure_queue = 'subsyscommand.locomotion.configure'

# Declare exchange from subsys
exchange_name = 'amq.topic'

channel = None

def on_connected(connection):
    # Called when we establish a connection to RabbitMQ server
    connection.channel(on_channel_open)

def on_channel_open(new_channel):
    # Called when establish a channel
    global channel
    channel = new_channel
    
  # declare and bind straight queue
    channel.queue_declare(on_straight_queue_declare, straight_queue, durable=True,
            exclusive=False, auto_delete=False)
    # declare and bind turn queue
    channel.queue_declare(on_turn_queue_declare, turn_queue, durable=True,
            exclusive=False, auto_delete=False)
  # declare and bind strafe queue
    channel.queue_declare(on_strafe_queue_declare, strafe_queue, durable=True,
            exclusive=False, auto_delete=False)
    # declare and bind strafe queue
    channel.queue_declare(on_configure_queue_declare, configure_queue, durable=True,
            exclusive=False, auto_delete=False)

def on_straight_queue_declare(frame):
    # Called when our queue is declared
    channel.queue_bind(callback=on_straight_queue_bind, queue=straight_queue, exchange=exchange_name, routing_key=straight_queue)

def on_turn_queue_declare(frame):
    # Called when our queue is declared
    channel.queue_bind(callback=on_turn_queue_bind, queue=turn_queue, exchange=exchange_name, routing_key=turn_queue)

def on_strafe_queue_declare(frame):
    # Called when our queue is declared
    channel.queue_bind(callback=on_strafe_queue_bind, queue=strafe_queue, exchange=exchange_name, routing_key=strafe_queue)

def on_configure_queue_declare(frame):
    # Called when our queue is declared
    channel.queue_bind(callback=on_configure_queue_bind, queue=configure_queue, exchange=exchange_name, routing_key=configure_queue)

def on_straight_queue_bind(frame):
    # Called when our queue is binded
    channel.basic_consume(handle_straight, queue=straight_queue, no_ack=True)

def on_turn_queue_bind(frame):
    # Called when our queue is binded
    channel.basic_consume(handle_turn, queue=turn_queue, no_ack=True)

def on_strafe_queue_bind(frame):
    # Called when our queue is binded
    channel.basic_consume(handle_strafe, queue=strafe_queue, no_ack=True)

def on_configure_queue_bind(frame):
    # Called when our queue is binded
    channel.basic_consume(handle_configure, queue=configure_queue, no_ack=True)

def handle_straight(channel, method, header, body):
    # Called when we receive messages from straight queue
    msg_in = messages_pb2.LocomotionControlCommandStraight()
    msg_in.ParseFromString(body)



    # else if the configuration is correct 
    # set the wheels rpm and timeout
    publish_wheel_rpm(Wheel.front_left, msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.front_right, msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.back_left, msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.back_right, msg_in.speed / 0.015959, msg_in.timeout)
    
def handle_turn(channel, method, header, body):
    # Called when we receive messages from turn queue
    msg_in = messages_pb2.LocomotionControlCommandTurn()
    msg_in.ParseFromString(body)

    # else if the configuration is correct
    publish_wheel_rpm(Wheel.front_left, msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.front_right, -msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.back_left, msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.back_right, -msg_in.speed / 0.015959, msg_in.timeout)
    
def handle_strafe(channel, method, header, body):
  # Called when we receive message from strafe queue
    msg_in = messages_pb2.LocomotionControlCommandStrafe()
    msg_in.ParseFromString(body)

    # else if the configuration is correct
    publish_wheel_rpm(Wheel.front_left, msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.front_right, -msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.back_left, -msg_in.speed / 0.015959, msg_in.timeout)
    publish_wheel_rpm(Wheel.back_right, msg_in.speed / 0.015959, msg_in.timeout)

def handle_configure(channel, method, header, body):
    # Called when we receive message from configure queue
    configuration = messages_pb2.LocomotionControlCommandConfigure()
    configuration.ParseFromString(body)

    if (configuration.target == 1):
        publish_wheel_pod_angle(Wheel.front_left, 20, configuration.timeout)
        publish_wheel_pod_angle(Wheel.front_right, 0, configuration.timeout)
        publish_wheel_pod_angle(Wheel.back_left, 0, configuration.timeout)
        publish_wheel_pod_angle(Wheel.back_right, 0, configuration.timeout)
    elif (configuration.target == 3):
        publish_wheel_pod_angle(Wheel.front_left, 60, configuration.timeout)
        publish_wheel_pod_angle(Wheel.front_right, 60, configuration.timeout)
        publish_wheel_pod_angle(Wheel.back_left, 60, configuration.timeout)
        publish_wheel_pod_angle(Wheel.back_right, 60, configuration.timeout)
    elif (configuration.target == 2):
        publish_wheel_pod_angle(Wheel.front_left, 97, configuration.timeout)
        publish_wheel_pod_angle(Wheel.front_right, 97, configuration.timeout)
        publish_wheel_pod_angle(Wheel.back_left, 97, configuration.timeout)
        publish_wheel_pod_angle(Wheel.back_right, 97, configuration.timeout)
    else:
        print('Bad config ' + str(configuration.target))
    
def publish_wheel_rpm(wheel, rpm, timeout):
    msg = messages_pb2.SpeedContolCommand()
    msg.rpm = rpm
    msg.timeout = timeout
    topic = 'motorcontrol.locomotion.' + wheel.name + '.wheel_rpm'
    channel.basic_publish(exchange=exchange_name,
            routing_key=topic,
            body=msg.SerializeToString())
    print(topic)

def publish_wheel_pod_angle(wheel, angle, timeout):
    msg = messages_pb2.PositionContolCommand()
    msg.position = angle
    msg.timeout = timeout
    topic = 'motorcontrol.locomotion.' + wheel.name + '.wheel_pod_pos'
    channel.basic_publish(exchange=exchange_name,
            routing_key=topic,
            body=msg.SerializeToString())


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
