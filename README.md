# rcam_ros2
Package for generating costmap with realsense camera\
To launch camera run: ros2 launch rcam_ros2 rcam.launch.py\
To run static transform publisher (needed for ros navigation to launch): ros2 run rcam_ros2 static_odom\
To generate costmap run: ros2 launch rcam_ros2 navigation.launch.py\
Visualize results with rviz config\