#!/bin/bash

# wait for reboot
sleep 6

# create a file for recording the log
LOG_FILE=/home/hzx/motor/log/start_ros_$(date +%Y%m%d_%H%M%S).log
mkdir -p $(dirname "$LOG_FILE")



# update the bash
source /opt/ros/melodic/setup.bash >> $LOG_FILE 2>&1
source /home/hzx/motor/devel/setup.bash >> $LOG_FILE 2>&1

# start the roscore
echo "Starting roscore..." >> $LOG_FILE
roscore >> $LOG_FILE 2>&1 &
ROSCORE_PID=$!
echo $ROSCORE_PID >> /home/hzx/motor/ros_pids.txt

# wait for roscore
sleep 3

# set and start the can
sudo /sbin/ip link set can0 up type can bitrate 1000000 >> $LOG_FILE 2>&1
sudo /sbin/ifconfig can0 up >> $LOG_FILE 2>&1
sleep 1
# start the can2usb
echo "Starting CAN nodes..." >> $LOG_FILE
roslaunch pubmotor can_motor.launch >> $LOG_FILE 2>&1 &
CAN_LAUNCH_PID=$!
echo $CAN_LAUNCH_PID >> /home/hzx/motor/ros_pids.txt

sleep 1

# start the Joy control node
echo "Starting JOY nodes..." >> $LOG_FILE
roslaunch wheeltec_joy WirelessJoy_crtl_agv.launch >> $LOG_FILE 2>&1 &
JOY_LAUNCH_PID=$!
echo $JOY_LAUNCH_PID >> /home/hzx/motor/ros_pids.txt

# start the main code
echo "Starting main node..." >> $LOG_FILE
rosrun pubmotor submuti >> $LOG_FILE 2>&1 &
MAIN_NODE_PID=$!
echo $MAIN_NODE_PID >> /home/hzx/motor/ros_pids.txt

echo "All nodes started at $(date)" >> $LOG_FILE

# 等待所有后台进程结束
wait $ROSCORE_PID $CAN_LAUNCH_PID $JOY_LAUNCH_PID $MAIN_NODE_PID
trap "kill $ROSCORE_PID $CAN_LAUNCH_PID $JOY_LAUNCH_PID $MAIN_NODE_PID; exit" SIGINT SIGTERM

