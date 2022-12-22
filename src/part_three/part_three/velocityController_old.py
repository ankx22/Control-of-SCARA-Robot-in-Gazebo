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

qo1 = 0.0
qo2 = 0.0
qo3 = 0.0
q1cur = 0.0
q2cur = 0.0
q3cur = 0.0
q1old = 0.0
q2old = 0.0
q3old = 0.0
timeold = -0.1
timecur = 0.0
p1Scalar = 2
p2Scalar = 2
p3Scalar = 2
d1Scalar = 0.5
d2Scalar = 0.5
d3Scalar = 0.5
q3const = 9.8
globalTime = 0
startTime = 0
recording = False

class controller(Node):

    def __init__(self):
        #Quality of Service profile was added to get rid of an error when creating publisher and subscriber
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        super().__init__('controller')
        self.publisher = self.create_publisher(Float64MultiArray, 'forward_effort_controller/commands',qos_profile)
        self.subscription = self.create_subscription(JointVelMsg, 'jointVel',self.jointVel_listener_callback,qos_profile)
        self.subscription = self.create_subscription(JointState, 'joint_states',
        self.jointState_listener_callback,qos_profile)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def jointVel_listener_callback(self, msg):
        global qo1, qo2, qo3
        global startTime
        global globalTime
        global recording
        global file
        qo1 = msg.q1
        qo2 = msg.q2
        qo3 = msg.q3
        recording = True
        self.get_logger().info('Beginning recording. Opening CSV file')
        file = open('log.csv','w')
        filewriter = csv.writer(file)
        filewriter.writerow(['Time','Current Position1','Current Position2','Current Position3','Reference Position1','Reference Position2','Reference Position3',])
        startTime = globalTime
        
    def jointState_listener_callback(self, msg):
        global q1cur
        global q2cur
        global q3cur
        global q1old
        global q2old
        global q3old
        global timeold
        global timecur
        global globalTime
        timeold = timecur
        timecur = msg.header.stamp.nanosec/1000000000
        globalTime = msg.header.stamp.nanosec/1000000000 + msg.header.stamp.sec
        q1old = q1cur
        q2old = q2cur
        q3old = q3cur
        q1cur = msg.velocity[0]
        q2cur = msg.velocity[1]
        q3cur = msg.velocity[2]
        
    def timer_callback(self):
        global qo1, qo2, qo3
        global timeold
        global timecur
        global q1cur
        global q2cur
        global q3cur
        global q1old
        global q2old
        global q3old
        global q3const
        global file
        global globalTime
        global recording
        if recording:
            filewriter = csv.writer(file)
            recordingTime = globalTime - startTime
            filewriter.writerow([recordingTime,q1cur,q2cur,q3cur,qo1,qo2,qo3])
            if recordingTime > 10:
            	recording = False
            	file.close()
            	self.get_logger().info('Finishing recording. Closing CSV file')
        if timeold > timecur:
           timecur += 1
        
        q1 = p1Scalar * (qo1-q1cur) + d1Scalar * (q1old-q1cur)/(timecur-timeold)
        q2 = p2Scalar * (qo2-q2cur) + d2Scalar * (q2old-q2cur)/(timecur-timeold)
        q3 = p3Scalar * (qo3-q3cur) + d3Scalar * (q3old-q3cur)/(timecur-timeold) + q3const  
        if timecur > 1:
           timecur -= 1  
        msg = Float64MultiArray()
        msg.data = [q1,q2,20.0]
        print(msg)
        self.publisher.publish(msg)
        self.i += 1

def main(args=None):
    print("Started")
    rclpy.init(args=args)
    server = controller()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
