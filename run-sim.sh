source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch "$(dirname "$(readlink -f "$0")")/launch/empty_world.launch.py"