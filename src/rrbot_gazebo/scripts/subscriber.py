#!/usr/bin/env python3
import math
import rclpy                                             #Import rclpy and math required libraries and dependencies 
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray        #built-in message type Float64MultiArray from std_msgs.msg
from sensor_msgs.msg import JointState            #built-in message type JointState from sensor_msgs.msg


class Subscriber(Node):                               #Create Subscriber Class, inheriting from Node Class

    def __init__(self):
        super().__init__('subscriber')                #Calling the constructor of Node class and giving it name of 'subscriber'
        self.sub = self.create_subscription(JointState, 'joint_states', self.chatter_callback, 10)     #Declaring that the subscriber message type and topic name it subscribes to
        self.publisher_ = self.create_publisher(Float64MultiArray, 'Pose', 10)           #Declaring the publisher message type and topic                                                       
        

    def chatter_callback(self, msg):
    
        q1=msg.position[0]           
        q2=msg.position[1]                                 #Breaking the received array into individual joint parameters
        d=msg.position[2]
        
        
        c1 = math.cos(q1)
        s1 = math.sin(q1)
        c2 = math.cos(q2)
        s2 = math.sin(q2)                             

        
       
        x11 = round(s1*s2-c1*c2,2)
        x12 = round(-(c1*s2+c2*s1),2)
        x13 = 0
        x14 = round(c1 + c1 *c2 - s1 * s2, 2)
        x21 = round(-c1 * s2 - c2 * s1,2)
        x22 = round(c1 * c2 - s1 * s2,2)
        x23 = 0   
        x24 = round(s1 +  s1 * c2 + c1 * s2, 2)                      #Calculating the entire transformation matrix 
        x31 = 0
        x32 = 0
        x33 = -1
        x34 = (d)
        x41 = 0
        x42 = 0 
        x43 = 0 
        x44 = 1

       
        P= Float64MultiArray()
        P.data=[x14 , x24 , x34]                                      #Publishing the Pose, i.e. Position for the Scara 
        self.publisher_.publish(P)
 

def main(args=None):
    rclpy.init(args=args)

    node = Subscriber()
    try:
        rclpy.spin(node)
        
       
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
