#!/usr/bin/env python3
import sys
import rclpy
import time
import csv
from math import cos, sin
from service_custom.srv import JointAngles
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

#global variables used to store the robot state, simulation time, and PD values
q1Vtarget = 0
q2Vtarget = 0
q3Vtarget = 0
q1Vcurrent = 0
q2Vcurrent = 0
q3Vcurrent = 0
q1Vold = 0
q2Vold = 0
q3Vold = 0
xVtarget = 0
yVtarget = 0
zVtarget = 0
q1Angle = 0
q2Angle = 0
q3Angle = 0
p1Scalar = 12.7
p2Scalar = 10
p3Scalar = 10
d1Scalar = 0
d2Scalar = 0
d3Scalar = 0
q3Const = 9.8
globalTime = 0
startTime = 0
timeold = -0.1
timecur = 0
recording = False




class Server(Node):

    def __init__(self):
        #Quality of Service profile was added to get rid of an error when creating publisher and subscriber
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        super().__init__('server')
        self.cli = self.create_service(JointAngles, 'velocityController', self.receive_request)
        self.subscription = self.create_subscription(JointState, 'joint_states',
        self.listener_callback,qos_profile)
        self.publisher = self.create_publisher(Float64MultiArray, 
        'forward_effort_controller/commands',qos_profile)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
    def receive_request(self, request, response):
        global xVtarget
        global yVtarget
        global zVtarget
        global startTime
        global globalTime
        global recording
        global file
        recording = True
        self.get_logger().info('Beginning recording. Opening CSV file') 

        file = open('log.csv','w')
        filewriter = csv.writer(file)
        filewriter.writerow(['Time','Velocity1','Velocity2','Velocity3',
        'Reference1','Refrence2','Reference 3',])
        startTime = globalTime
        xVtarget = request.q1
        yVtarget = request.q2
        zVtarget = request.q3
        response.response = 'Goal Velocities Received'
        return response
        
    def listener_callback(self,msg):
        global q1Vcurrent
        global q2Vcurrent
        global q3Vcurrent
        global q1Vold
        global q2Vold
        global q3Vold
        global q1Angle
        global q2Angle
        global q3Angle
        global timeold
        global timecur
        global globalTime
        globalTime = msg.header.stamp.nanosec/1000000000 + msg.header.stamp.sec
        timeold = timecur
        timecur = msg.header.stamp.nanosec/1000000000
        q1Vold = q1Vcurrent
        q2Vold = q2Vcurrent
        q3Vold = q3Vcurrent
        q1Vcurrent = msg.velocity[0]
        q2Vcurrent = msg.velocity[1]
        q3Vcurrent = msg.velocity[2]
        q1Angle = msg.position[0]
        q2Angle = msg.position[1]
        q3Angle = msg.position[2]
        
    def timer_callback(self):
        global xVtarget
        global yVtarget
        global zVtarget
        global q1Vcurrent
        global q2Vcurrent
        global q3Vcurrent
        global q1Vold
        global q2Vold
        global q3Vold
        global q1Angle
        global q2Angle
        global q3Angle
        global p1Scalar, p2Scalar, p3Scalar, q3Const
        global timeold
        global timecur
        global globalTime
        global recording

        if q2Angle != 0:
           q1Vtarget = (D()*xVtarget - B()*yVtarget) /(A()*D()- B()*C())
           q2Vtarget = (-C()*xVtarget + A()*yVtarget)/(A()*D()- B()*C())
           if timeold > timecur:
              timecur += 1
           q1effort = p1Scalar * (q1Vtarget-q1Vcurrent)# + d1Scalar * (q1old-q1cur)/(timecur-timeold)
           q2effort = p2Scalar * (q2Vtarget-q2Vcurrent)# + d2Scalar * (q2old-q2cur)/(timecur-timeold)
           q3effort = p3Scalar * (q3Vtarget-q3Vcurrent) + q3Const# + d3Scalar * (q3old-q3cur)/(timecur-timeold)  
           if timecur > 1:
              timecur -= 1
           msg = Float64MultiArray()
           msg.data = [q1effort,q2effort,q3effort]
           print(msg.data)
           self.publisher.publish(msg)
           self.i += 1
        if recording:
           filewriter = csv.writer(file)
           recordingTime = globalTime - startTime
           filewriter.writerow([recordingTime,q1Vcurrent,q2Vcurrent,q3Vcurrent,q1Vtarget,q2Vtarget,q3Vtarget])
           if recordingTime > 10:
              recording = False
              file.close()
              self.get_logger().info('Finishing recording. Closing CSV file')

def A(args=None):
    global q1Angle, q2Angle
    return -sin(q1Angle)-sin(q1Angle+q2Angle)  

def B(args=None):
    global q1Angle, q2Angle
    return -sin(q1Angle+q2Angle) 

def C(args=None):
    global q1Angle, q2Angle
    return cos(q1Angle)+cos(q1Angle+q2Angle) 

def D(args=None):
    global q1Angle, q2Angle
    return cos(q1Angle+q2Angle) 
   
def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
