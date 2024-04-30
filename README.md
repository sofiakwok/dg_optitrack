Ros2 node for subscribing to position/attitude estimates with optitrack motion capture system. Run on Ubuntu 20 with Ros2 Foxy. 
To run code: 
1. Follow instructions for getting mocap4 for ros2: https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack
2. Make sure to activate mocap4r2_optitrack_driver node
3. Source /opt/ros/foxy/setup.bash
4. Source mocap4r2_ws/install/setup.bash (needed to get RigidBodies data type)
5. Source dg_optitrack/install/setup.bash
6. ros2 run dg_optitrack subscriber

To rebuild repository: ```colcon build``` in root directory (dg_optitrack)
