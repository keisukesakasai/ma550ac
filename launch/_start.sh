source /opt/ros/kinetic/setup.bash
source ~/ros/devel/setup.bash
export ROS_IP=172.20.0.20
export ROS_MASTER_URI=http://172.20.0.20:11311
export ROSLAUNCH_SSH_UNKNOWN=1
roslaunch _start.launch
