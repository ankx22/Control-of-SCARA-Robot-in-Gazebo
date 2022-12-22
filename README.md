# Control-of-SCARA-Robot-in-Gazebo

# Install 
Environment: Ubuntu 22.04 + ROS2 Humble Hawksbill

Build Instructions
Clone this repository under catkin_ws/src/
Compile under catkin_ws: colcon build
Make all python files as executables. 


# Part 1
1. Spawn the Robot
For part 1, we spawn the robot according to the given configuration of SCARA robot which is RRP. We add a prismatic joint to the end of the 2nd joint.The joints are rotating along the positive motion axis. Using these parameters we calculat the forward and inverse kinematics for the robot. Command for launching the robot in gazebo is given as- `ros2 launch rrbot_gazebo rrbot_world.launch.py`

 ![rrp1](https://user-images.githubusercontent.com/80807952/209065316-d49fb42f-0c8d-4648-97c8-c2dc7d02a038.png)
 
 
2. Forward Kinematics
We define a node that has one subscriber which subscribes to the joint states, and a publisher, which after  converting the joint values to pose, publishes the end effector pose. The terminal commands to run the node and see the Pose are- `ros2 run rrbot_gazebo subscriber.py`  and `ros2 topic echo Pose`. Given below is a sample image of the forward kinematics. Command `ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{data:[1,1,1]}'` is used for moving the robot, given input joint values. 
![rrp2](https://user-images.githubusercontent.com/80807952/209066657-3181d693-9ef5-49d2-8f5d-af26ed8d4683.png)


3. Inverse Kinematics
First step is to make a custom service file. Then, Inverse Kinematics was accomplished using service server-service client. The command used to call the service is `ros2 run rrbot_inverse1 service`. Calling the service using the command `ros2 service call inversekinematics service_custom/srv/IKinematics "{x: 0.5,y: 1,z: 0.5}"`, returns back the joint values, thus finishing inverse kinematics. The values were validated by using the command `ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{data:[1,1,1]}'`, and checking if the robot moves to the particular end effector position. 
![rrp3](https://user-images.githubusercontent.com/80807952/209067413-efc876ad-d9dd-49fd-b2b2-d11c9ab84d6b.png)

# Part 2

 

