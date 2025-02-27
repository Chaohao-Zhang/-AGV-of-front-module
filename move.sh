#!/bin/bash

LOG_FILE=/home/hzx/motor/log/move_$(date +%Y%m%d_%H%M%S).log

# update the bash
source /opt/ros/melodic/setup.bash >> $LOG_FILE 2>&1
source /home/hzx/motor/devel/setup.bash >> $LOG_FILE 2>&1

# start the main code
echo "Starting main node..." >> $LOG_FILE
stdbuf -o L rosrun move moving >> $LOG_FILE 2>&1 &
MOVE_NODE_PID=$!
echo $MOVE_NODE_PID >> /home/hzx/motor/ros_pids.txt

# 等待所有后台进程结束
wait $MOVE_NODE_PID
trap "kill $MOVE_NODE_PID; exit" SIGINT SIGTERM