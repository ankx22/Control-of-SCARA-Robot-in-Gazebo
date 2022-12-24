# Control-of-SCARA-Robot-in-Gazebo

# Install 
Environment: Ubuntu 22.04 + ROS2 Humble Hawksbill

Build Instructions : 

Clone this repository under catkin_ws/src/
Compile under catkin_ws: colcon build
Make all python files as executables. 


# Part 1
1. Spawn the Robot
For part 1, we spawn the robot according to the given configuration of SCARA robot which is RRP. We add a prismatic joint to the end of the 2nd joint.The joints are rotating along the positive motion axis. Using these parameters we calculat the forward and inverse kinematics for the robot. Command for launching the robot in gazebo is given as- `ros2 launch rrbot_gazebo rrbot_world.launch.py`

 ![rrp1](https://user-images.githubusercontent.com/80807952/209065316-d49fb42f-0c8d-4648-97c8-c2dc7d02a038.png)
 
 
2. Forward Kinematics
We define a node that has one subscriber which subscribes to the joint states, and a publisher, which after  converting the joint values to pose, publishes the end effector pose. The terminal commands to run the node and see the Pose are- `ros2 run rrbot_gazebo subscriber.py`  and `ros2 topic echo Pose`. Given below is a sample image of the forward kinematics. Command `ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{data:[1,1,1]}'` is used for moving the robot, given input joint values. You can give any values for joint values, not necessarily [1,1,1].
![rrp2](https://user-images.githubusercontent.com/80807952/209066657-3181d693-9ef5-49d2-8f5d-af26ed8d4683.png)


3. Inverse Kinematics
First step is to make a custom service file. Then, Inverse Kinematics was accomplished using service server-service client. The command used to call the service is `ros2 run rrbot_inverse1 service`. Calling the service using the command `ros2 service call inversekinematics service_custom/srv/IKinematics "{x: 0.5,y: 1,z: 0.5}"`, returns back the joint values, thus finishing inverse kinematics. The values were validated by using the command `ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{data:[1,1,1]}'`, and checking if the robot moves to the particular end effector position. 
![rrp3](https://user-images.githubusercontent.com/80807952/209067413-efc876ad-d9dd-49fd-b2b2-d11c9ab84d6b.png)

# Part 2
Part 2 of this project is to implement and tune a PD position controller. The approach of fixing all but one joint was used in order to tune the PD controllers. 

1. Custom Generate a service file, with float values for the three reference joint values, and a string for the response. Command for launching the robot in gazebo is given as- `ros2 launch rrbot_gazebo rrbot_world.launch.py`

2. The reference joint value is passed using the command - `ros2 service call /pdController service_custom/srv/JointAngles "{q1: -4,q2: 3,q3: 0.8}"`. You may give any value for the reference joint values. Then, the node having the service-client, subscriber to take the current joint states, and publisher to publish back the required joint efforts is called using the command - `ros2 run part_two service`.

3. As explained above, tuning the PD gains can be done by fixing all but one joint, in the URDF file, in rrbot_description.urdf. 


# Part 3
Part 3 of this project is to implement a node for forward and inverse velocity kinematics. To run the node, use the command `ros2 run part_three service`. You can run the command lines in order to implement the velocity kinematics,
`ros2 service call /CartToJointVelocities service_custom/srv/CartVelocities "{x: 0,y: 1,z: 0.5}"`. 
`ros2 service call /JointToCartVelocities service_custom/srv/JointVelocities "{q1: 1,q2: -1,q3: 0.5}"`. 

Again, you can pass any values of your own choice. 
![rrp4](https://user-images.githubusercontent.com/80807952/209433862-aa752295-f158-4bca-9bbe-0564b092fa5b.png)

For the PD controller, the values were tuned again accordingly by keeping only one joint as moveable at a time. 

