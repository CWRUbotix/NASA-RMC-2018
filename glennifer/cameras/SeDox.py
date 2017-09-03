import cv2
import numpy as np
import pika
import sys
credentials = pika.PlainCredentials('guest', 'guest')
parameters = pika.ConnectionParameters('localhost',  5672, '/', credentials=credentials)

#host_input_name = raw_input("Please Connect to a Host ")
#pika.ConnectionParameters(host = host_input_name)
#Sends a message
def send(msg):
    connection = pika.BlockingConnection(parameters)
    channel = connection.channel()
    channel.queue_declare(queue = 'q03')
    channel.basic_publish(exchange = '',  routing_key = 'q03',  body = msg)

#Camera
cam1 = cv2.VideoCapture(0)
cam1.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
cam1.set(cv2.CAP_PROP_FRAME_HEIGHT,  240)

#Main Loop
while(True):
    #Camera One Updates
    ret,  frame1 = cam1.read()
    #cam_one_raw = cv2.cvtColor(frame1,  cv2.COLOR_BGR2RGB)
    img = cv2.imencode('.png',  frame1)[1].tostring()
    send(img)
    
    
