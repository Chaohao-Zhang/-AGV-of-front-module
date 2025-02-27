#!/bin/bash

# 读取并终止所有进程
if [ -f /home/hzx/motor/ros_pids.txt ]; then
    while IFS= read -r pid
    do
        echo "Killing process $pid"
        kill $pid
    done < /home/hzx/motor/ros_pids.txt

    # 删除PID文件
    rm /home/hzx/motor/ros_pids.txt
else
    echo "PID file not found"
fi
