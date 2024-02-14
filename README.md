# Użyłam 3 poleceń:

ros2 launch diagnostics_analyzer my_publisher_launch.yaml
ros2 topic pub /cpu_utilization std_msgs/Float32 "data: 0.9"
ros2 topic echo /diagnostics

