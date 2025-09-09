#!/bin/bash

# Step 1: Source your custom ROS workspace
echo "Step 1: Sourcing your custom ROS workspace..."
if [ $? -eq 0 ]; then
echo "Custom ROS workspace sourced successfully."
else
echo "Failed to source custom ROS workspace. Exiting."
exit 1
fi

# Step 2: Launch multiple ROS launch files in new terminal windows
echo "Step 2: Launching multiple ROS launch files..."

gnome-terminal --tab  -- bash -c "roslaunch mavros px4.launch; exec bash" &
sleep 2 # Give the launch file some time to start

gnome-terminal --tab -- bash -c "source ~/driver_ws/devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash" &
sleep 4 # Give the launch file some time to start

gnome-terminal --tab  -- bash -c "source ~/driver_ws/devel/setup.bash; roslaunch lsdc_slam mapping_livox.launch; exec bash" &
sleep 2 # Give the launch file some time to start

# gnome-terminal --tab  -- bash -c "rosrun rqt_topic rqt_topic; exec bash" &
# sleep 2 # Give the launch file some time to start

#gnome-terminal --tab  -- bash -c "rosbag record -a; exec bash" &
#sleep 2 # Give the launch file some time to start

# gnome-terminal --tab  -- bash -c "source ~/drone_op_ws/devel/setup.bash; rosrun task_manager task_distr; exec bash" &
# sleep 2 # Give the launch file some time to start


# gnome-terminal --tab  -- bash -c "source ~/drone_op_ws/devel/setup.bash; rosrun task_manager task_parser; exec bash" &
# sleep 2 # Give the launch file some time to start
