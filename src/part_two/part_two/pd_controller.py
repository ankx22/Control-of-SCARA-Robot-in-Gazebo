#!/usr/bin/env python3
import sys
import rclpy
import time
import csv
from service_custom.srv import JointAngles
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

#global variables used to store the robot state, simulation time, and PD values
qo1 = 0
qo2 = 0
qo3 = 0
q1cur = 0
q2cur = 0
q3cur = 0
q1old = 0
q2old = 0
q3old = 0
timeold = -0.1
timecur = 0
p1Scalar = 5
p2Scalar = 5
p3Scalar = 5
d1Scalar = 10
d2Scalar = 10
d3Scalar = 8
q3const = 9.8
globalTime = 0
startTime = 0
recording = False


class Server(Node):

    def __init__(self):
        #Quality of Service profile was added to get rid of an error when creating publisher and subscriber
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5)
        super().__init__('server')
        #Create service that takes in reference joint angles, publisher that sends joint efforts to gazebo, and subscriber that listens to joint states from gazebo
        self.cli = self.create_service(JointAngles, 'pdController', self.receive_request)
        self.subscription = self.create_subscription(JointState, 'joint_states',
        self.listener_callback,qos_profile)
        self.publisher = self.create_publisher(Float64MultiArray, 'forward_effort_controller/commands',qos_profile)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
    def receive_request(self, request, response):
        global qo1
        global qo2
        global qo3
        global startTime
        global globalTime
        global recording
        global file
        recording = True
        self.get_logger().info('Beginning recording. Opening CSV file')
        file = open('log.csv','w')
        filewriter = csv.writer(file)
        filewriter.writerow(['Time','Current Position1','Current Position2','Current Position3','Reference Position1','Reference Position2','Reference Position3',])
        #startTime = globalTime
        qo1 = request.q1
        qo2 = request.q2
        qo3 = request.q3
        response.response = 'Goal Angles Received'
        return response
        
    def listener_callback(self,msg):
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
        q1cur = msg.position[0]
        q2cur = msg.position[1]
        q3cur = msg.position[2]
        
    def timer_callback(self):
        global timeold
        global timecur
        global q1cur
        global q2cur
        global q3cur
        global q1old
        global q2old
        global q3old
        global qo1
        global qo2
        global qo3
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
        msg.data = [q1,q2,q3]
        print(msg.data)
        self.publisher.publish(msg)
        self.i += 1
       
def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
