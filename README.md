# sawyer_metc_gripper

This ROS package is a Sawyer ClickSmart gripper controller. 

It define a message GripperControl (boolean) to control the gripper on or off. 

Usage: 
$ rosrun sawyer_metc_gripper gripper_control.py

In another program, define a publisher to publish the GripperControl (1 or 0) to ROS topic. 
