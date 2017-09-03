import cv2
import numpy as np
import pika
import sys
import time
#import yaml

#with open('config/connection.yml') as connection_config_file:
    #connection_config = yaml.safe_load(connection_config_file)
    #amqp_server_addr = connection_config['server-addr']
    #amqp_server_user = connection_config['server-user']
    #amqp_server_pass = connection_config['server-pass']
    #amqp_exchange_name = connection_config['exchange-name']

#credentials = pika.PlainCredentials(amqp_server_user, amqp_server_pass)
#parameters = pika.ConnectionParameters(amqp_server_addr,  5672, '/', credentials=credentials)

connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
channel = connection.channel()
channel.exchange_declare(exchange='amq.topic', type = 'topic', durable = True)

#Sends a message
def send1(msg):
    channel.basic_publish(exchange = 'amq.topic',  routing_key = 'camera.one',  body = msg)
    
def send2(msg):
    channel.basic_publish(exchange = 'amq.topic',  routing_key = 'camera.two',  body = msg)
    
def send3(msg):
    channel.basic_publish(exchange = 'amq.topic',  routing_key = 'camera.three',  body = msg)
    
def send4(msg):
    channel.basic_publish(exchange = 'amq.topic',  routing_key = 'camera.four',  body = msg)
    
def send5(msg):
    channel.basic_publish(exchange = 'amq.topic', routing_key = 'camera.five', body = msg)
    
#Camera Capture
#Change Capture Device Index to correspinding index
#for corresponding camera
cam1 = cv2.VideoCapture(0)
cam1.set(3, 240)
cam1.set(4, 160)
#cam1.set(cv2.cv.CV_CAP_PROP_FPS, 15)
cam2 = cv2.VideoCapture(1)
cam2.set(3, 240)
cam2.set(4, 160)
cam3 = cv2.VideoCapture(2)
cam3.set(3, 240)
cam3.set(4, 160)
cam4 = cv2.VideoCapture(3)
cam4.set(3, 240)
cam4.set(4, 160)
cam5 = cv2.VideoCapture(4)
cam5.set(3, 240)
cam5.set(4, 160)
def getCam1():
    if (cam1.isOpened() is True):
        ret,  frame1 = cam1.read()
        #imr1 = cv2.cvtColor(frame1,  cv2.COLOR_BGR2RGB)
        #img1 = str(bytearray(cv2.imencode('.png',  frame1)[1].flatten().tolist()))
	img1 = cv2.imencode('.jpeg',  frame1, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1].tostring()
        #print(len(img1))
        send1(img1)
    else:
        c= 1
        
def getCam2():
    if (cam2.isOpened() is True):
        ret,  frame2 = cam2.read()
        #imr2 = cv2.cvtColor(frame2,  cv2.COLOR_BGR2RGB)
        #img2 = str(bytearray(cv2.imencode('.png',  frame2)[1].flatten().tolist()))
        img2 = cv2.imencode('.jpeg',  frame2, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1].tostring()
        send2(img2)
    else:
        c= 1
        
def getCam3():
    if (cam3.isOpened() is True):
        ret,  frame3 = cam3.read()
        #imr3 = cv2.cvtColor(frame3,  cv2.COLOR_BGR2RGB)
        img3 = cv2.imencode('.jpeg',  frame3, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1].tostring()
        #img3 = str(bytearray(cv2.imencode('.png',  frame3)[1].flatten().tolist()))
        send3(img3)
    else:
         c= 1
        
def getCam4():
    if (cam4.isOpened() is True):
        ret,  frame4 = cam4.read()
        #imr4 = cv2.cvtColor(frame4,  cv2.COLOR_BGR2RGB)
        img4 = cv2.imencode('.jpeg',  frame4, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1].tostring()
        #img4 = str(bytearray(cv2.imencode('.png',  frame4)[1].flatten().tolist()))
        send4(img4)
    else:
         c= 1
        
def getCam5():
    if (cam5.isOpened() is True):
        ret,  frame5 = cam5.read()
        #imr5 = cv2.cvtColor(frame5,  cv2.COLOR_BGR2RGB)
        img5 = cv2.imencode('.jpeg',  frame5, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1].tostring()
        #print(img5)
        #print(len(img5))
        #img5 = str(bytearray(cv2.imencode('.jpeg',  frame5)[1].flatten().tolist()))
        #print(len(img5))
        #print(len(img5))
        send5(img5)
    else:
         c= 1

#Main Loop
while(True):
    #Camera One Update
    getCam1()
    #Camera Two Update
    getCam2()
    #Camera Three Update
    getCam3()
    #Camera Four Update
    getCam4()
    #Camera Five Update
    getCam5()
    time.sleep(0.2)
