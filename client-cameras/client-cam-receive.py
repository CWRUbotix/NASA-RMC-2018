import cv2
import pika
import numpy as np

#Change demo to guest or local host, or another user, password tuple
#Change ip address to the one for the server running the receive code
#172.20.93.234
#192.16.80.200
#192.168.0.200
#172.20.36.235
credentials = pika.PlainCredentials('guest', 'guest')
parameters = pika.ConnectionParameters('192.168.0.200', 5672,  '/', credentials=credentials)
connection = pika.BlockingConnection(parameters)
#connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
channel = connection.channel()
channel.exchange_declare(exchange='amq_topic',  type = 'topic')
queue1 = channel.queue_declare(exclusive=True)
q1_name = queue1.method.queue
channel.queue_bind(exchange = 'amq_topic', queue = q1_name, routing_key = 'camera.one')
queue2 = channel.queue_declare(exclusive=True)
q2_name = queue2.method.queue
channel.queue_bind(exchange = 'amq_topic', queue = q2_name, routing_key = 'camera.two')
queue3 = channel.queue_declare(exclusive=True)
q3_name = queue3.method.queue
channel.queue_bind(exchange = 'amq_topic', queue = q3_name, routing_key = 'camera.three')
queue4 = channel.queue_declare(exclusive=True)
q4_name = queue4.method.queue
channel.queue_bind(exchange = 'amq_topic', queue = q4_name, routing_key = 'camera.four')
queue5 = channel.queue_declare(exclusive=True)
q5_name = queue5.method.queue
channel.queue_bind(exchange = 'amq_topic', queue = q5_name, routing_key = 'camera.five')

fourcc1 = cv2.cv.CV_FOURCC(*'DIVX')
fourcc2 = cv2.cv.CV_FOURCC(*'DIVX')
fourcc3 = cv2.cv.CV_FOURCC(*'DIVX')
fourcc4 = cv2.cv.CV_FOURCC(*'DIVX')
fourcc5 = cv2.cv.CV_FOURCC(*'DIVX')

out1 = cv2.VideoWriter('output1.avi', fourcc1, 20.0, (640,480))
out2 = cv2.VideoWriter('output2.avi', fourcc1, 20.0, (640,480))
out3 = cv2.VideoWriter('output3.avi', fourcc1, 20.0, (640,480))
out4 = cv2.VideoWriter('output4.avi', fourcc1, 20.0, (640,480))
out5 = cv2.VideoWriter('output5.avi', fourcc1, 20.0, (640,480))
#Camera Windows
cv2.namedWindow('Cam1')
cv2.namedWindow('Cam2')
cv2.namedWindow('Cam3')
cv2.namedWindow('Cam4')
cv2.namedWindow('Cam5')
#cv2.startWindowThread()
def callback1(ch,  method,  properties, body):
    if (body is None):
        return None
    else: 
        nparr = np.fromstring(body,  np.uint8)
        img = cv2.imdecode(nparr,  cv2.CV_LOAD_IMAGE_COLOR)
        #out1.write(img)
        cv2.imshow('Cam1', img)
        cv2.waitKey(15)
        
def callback2(ch,  method,  properties, body):
    if (body is None):
        return None
    else: 
        nparr = np.fromstring(body,  np.uint8)
        img = cv2.imdecode(nparr,  cv2.CV_LOAD_IMAGE_COLOR)
        #out2.write(img)
        cv2.imshow('Cam2', img)
        cv2.waitKey(15)
        
def callback3(ch,  method,  properties, body):
    if (body is None):
        return None
    else: 
        nparr = np.fromstring(body,  np.uint8)
        img = cv2.imdecode(nparr,  cv2.CV_LOAD_IMAGE_COLOR)
        #out3.write(img)
        cv2.imshow('Cam3', img)
        cv2.waitKey(15)      
      
def callback4(ch,  method,  properties, body):
    if (body is None):
        return None
    else: 
        nparr = np.fromstring(body,  np.uint8)
        img = cv2.imdecode(nparr,  cv2.CV_LOAD_IMAGE_COLOR)
        #out4.write(img)
        cv2.imshow('Cam4', img)
        cv2.waitKey(15)
        
def callback5(ch,  method,  properties, body):
    if (body is None):
        return None
    else: 
        nparr = np.fromstring(body,  np.uint8)
        img = cv2.imdecode(nparr,  cv2.CV_LOAD_IMAGE_COLOR)
        #out5.write(img)
        cv2.imshow('Cam5', img)
        cv2.waitKey(15)
    
#Consume
channel.basic_consume(callback1,  queue = q1_name,  no_ack = True)
channel.basic_consume(callback2,  queue = q2_name,  no_ack = True)
channel.basic_consume(callback3,  queue = q3_name,  no_ack = True)
channel.basic_consume(callback4,  queue = q4_name,  no_ack = True)
channel.basic_consume(callback5,  queue = q5_name,  no_ack = True)
channel.start_consuming()
