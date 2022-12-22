#!/usr/bin/env python3
import sys
import rclpy
import time
import csv
from math import sin, cos
from service_custom.srv import JointVelocities, CartVelocities
from service_custom.msg import JointVelMsg
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

q1 = 0.0
q2 = 0.0
x = 0.0
y = 0.0
z = 0.0

class Server(Node):

    def __init__(self):
        #Quality of Service profile was added to get rid of an error when creating publisher and subscriber
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        super().__init__('server')
        
        self.cli = self.create_service(JointVelocities, 'JointToCartVelocities', self.jointToCart)
        self.cli = self.create_service(CartVelocities, 'CartToJointVelocities', self.cartToJoint)
        self.subscription = self.create_subscription(JointState, 'joint_states',self.listener_callback,qos_profile)

    def jointToCart(self, request, response):
        global q1, q2
        response.x = A()*request.q1 + B()*request.q2
        response.y = C()*request.q1 + D()*request.q2
        response.z = request.q3
        return response
    
    def cartToJoint(self, request, response):
        global q1, q2
        response.q1 = (D()*request.x - B()*request.y)/(A()*D()- B()*C())
        response.q2 = (-C()*request.x + A()*request.y)/(A()*D()- B()*C())
        response.q3 = request.z
        return response
    
    def listener_callback(self, msg):
        global q1, q2
        q1 = msg.position[0]
        q2 = msg.position[1]
        
        
def A(args=None):
    global q1, q2
    return -sin(q1)-sin(q1+q2)  

def B(args=None):
    global q1, q2
    return -sin(q1+q2) 

def C(args=None):
    global q1, q2
    return cos(q1)+cos(q1+q2) 

def D(args=None):
    global q1, q2
    return cos(q1+q2) 
       
def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
