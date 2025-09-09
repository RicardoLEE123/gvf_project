#!/bin/bash

# Step 1: Source your custom ROS workspace
echo "Step 1: Sourcing your custom ROS workspace..."
if [ $? -eq 0 ]; then
echo "Custom ROS workspace sourced successfully."
else
echo "Failed to source custom ROS workspace. Exiting."
exit 1
fi

echo "Step 2: Launching multiple ROS launch files..."

gnome-terminal --tab  -- bash -c "roslaunch mavros px4.launch; exec bash" &
sleep 2 

gnome-terminal --tab -- bash -c "source ~/driver_ws/devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash" &
sleep 4 

gnome-terminal --tab  -- bash -c "source ~/driver_ws/devel/setup.bash; roslaunch lsdc_slam mapping_livox.launch; exec bash" &
sleep 2 

gnome-terminal --tab  -- bash -c "source ~/gvfproject/devel/setup.bash; roslaunch bspline_race gvf.launch; exec bash" &
sleep 2 

gnome-terminal --tab  -- bash -c "cd ~/record_bag; echo './record.sh'; exec bash" &
