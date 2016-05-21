#/bin/bash

rosbag record /camera/left/camera_info /camera/left/image_raw /camera/right/camera_info /camera/right/image_raw /cmd_vel /imu/data /husky_velocity_controller/cmd_vel /husky_velocity_controller/odom /mavros/imu/data /mavros/imu/mag /mavros/imu/temperature
