# real camera
Package for generating costmap with realsense camera\
To launch camera run: ros2 launch rcam_ros2 rcam.launch.py\
To run static transform publisher (needed for ros navigation to launch): ros2 run rcam_ros2 static_odom\  
To generate costmap run: ros2 launch rcam_ros2 cost.launch.py\
Visualize results with rviz config
# simulator
To launch simulator run: ros2 launch rcam_sim world.launch.py\
To launch building of costmap run: ros2 launch rcam_ros2 cost.launch.py use_sim_time:=true\
To control robot with the keyboard run: ros2 run rcam_sim teleop.py\
To start costmap analizer run: ros2 run rcam_sim costmap_analizer.py\
