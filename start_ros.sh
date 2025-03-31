#!/bin/bash

# 等待系统初始化
sleep 6

# 日志文件配置
LOG_FILE="/home/hzx/motor/log/start_ros_$(date +'%Y%m%d_%H%M%S').log"
mkdir -p "$(dirname "$LOG_FILE")"

# 全局输出重定向（同时显示终端和记录日志）
exec > >(tee -a "$LOG_FILE") 2>&1

# 初始化ROS环境
source /opt/ros/melodic/setup.bash
source /home/hzx/motor/devel/setup.bash

# 精确PID获取函数（增加调试输出）
get_ros_pid() {
    local pattern="$1"
    local timeout=5
    local pid=0
    
    echo "DEBUG: 开始查找进程模式: $pattern" >&2
    for ((i=0; i<timeout*2; i++)); do
        pid=$(pgrep -f "$pattern" | head -n1)
        [ -n "$pid" ] && break
        sleep 0.5
    done
    
    if [ -n "$pid" ] && ps -p $pid > /dev/null; then
        echo "DEBUG: 找到进程 PID: $pid (模式: $pattern)" >&2
        echo $pid
    else
        echo "DEBUG: 未找到进程 (模式: $pattern)" >&2
        echo "0"
    fi
}

# 启动roscore
echo "[$(date '+%T')] 启动roscore核心..."
{
    exec roscore
} &
ROSCORE_PID=$(get_ros_pid "roscore")
[ "$ROSCORE_PID" != "0" ] && echo $ROSCORE_PID > /home/hzx/motor/ros_pids.txt

# 等待roscore就绪
until rostopic list >/dev/null 2>&1; do
    sleep 0.5
done
echo "[$(date '+%T')] roscore准备就绪"

# 配置CAN接口
echo "[$(date +"%T")] 初始化CAN接口..."
sudo ip link set can0 up type can bitrate 1000000 >> $LOG_FILE 2>&1
sudo ifconfig can0 up >> $LOG_FILE 2>&1
sleep 0.5

# CAN节点启动
echo "[$(date '+%T')] 初始化CAN通信..."
{
    exec stdbuf -oL roslaunch pubmotor can_motor.launch
} &
CAN_PID=$(get_ros_pid "can_motor.launch")
[ "$CAN_PID" != "0" ] && echo $CAN_PID >> /home/hzx/motor/ros_pids.txt

# 手柄控制节点
sleep 1
echo "[$(date '+%T')] 启动手柄控制..."
{
    exec stdbuf -oL roslaunch wheeltec_joy WirelessJoy_crtl_agv.launch
} &
JOY_PID=$(get_ros_pid "WirelessJoy_crtl_agv.launch")
[ "$JOY_PID" != "0" ] && echo $JOY_PID >> /home/hzx/motor/ros_pids.txt

# 主控节点（C++）
sleep 0.5
echo "[$(date '+%T')] 启动主控程序..."
{
    exec stdbuf -oL rosrun pubmotor submuti
} &
MAIN_PID=$(get_ros_pid "submuti")
[ "$MAIN_PID" != "0" ] && echo $MAIN_PID >> /home/hzx/motor/ros_pids.txt

# TCP节点（Python）
sleep 0.5
echo "[$(date '+%T')] 启动TCP服务..."
{
    exec python3 -u /home/hzx/motor/src/TCP_Twist/src/SendSpeed.py
} &
TCP_PID=$(get_ros_pid "SendSpeed.py")
[ "$TCP_PID" != "0" ] && echo $TCP_PID >> /home/hzx/motor/ros_pids.txt

# 完成提示
echo "[$(date '+%T')] 系统启动完成 | 记录PID: $ROSCORE_PID,$CAN_PID,$JOY_PID,$MAIN_PID,$TCP_PID"

# 信号捕获和清理
trap "kill -TERM $ROSCORE_PID $CAN_PID $JOY_PID $MAIN_PID $TCP_PID 2>/dev/null; wait" SIGINT SIGTERM
wait