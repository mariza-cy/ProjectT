source /opt/ros/humble/setup.bash 
source ./install/setup.bash 
export VEHICLE_NAME=duckie05
export USER_NAME=duckie05
chmod a+x ./install/tof_reader/lib/tof_reader/autotosdrive.py 
ls -al ./install/tof_reader/lib/tof_reader/autotosdrive.py 
ros2 run tof_reader autotosdrive.py