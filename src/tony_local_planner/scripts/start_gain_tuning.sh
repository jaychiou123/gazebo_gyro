#!/bin/bash

display_help() {
    echo "Usage: rosrun tony_local_planner start_gain_tuning.sh [option...]" >&2
    echo
    echo "   -h, --help                     Show this help message and exit"
    echo "   -l, --local-ip=LOCAL_IP        Set the ip address of your local pc"
    echo "   -m, --machine-ip=MACHINE_IP    Set the ip address of the remote host"
    echo "   -s, --setup-script=SCRIP_DIR   Set the directory of the setup script for remote launch process to execute during startup"
    echo "   -g, --global-planner=PLANNER   Specify the global planner for plan generation, default global_planner/GlobalPlanner"
    echo "   -p, --port=PORT_NUM            Specify the port number that the master is at, default 11311"
    exit 1
}

options=`getopt -o hl:m:s:g:p: --long help,local-ip:,machine-ip:,setup-script:,global-planner:,--port -- "$@"`
eval set -- "$options"

LOCAL_IP="127.0.0.1"
MACHINE_IP="localhost"
SCRIPT_DIR="/home/gyro/remote_launch.sh"
GLOBAL_PLANNER="global_planner/GlobalPlanner"
PORT="11311"

while true; do
    case "$1" in
        -h | --help ) display_help; shift;;
        -l | --local-ip ) LOCAL_IP="$2"; shift 2;;
        -m | --machine-ip ) MACHINE_IP="$2"; shift 2;;
        -s | --setup-script ) SCRIPT_DIR="$2"; shift 2;;
        -g | --global-planner ) GLOBAL_PLANNER="$2"; shift 2;;
        -p | --port ) PORT="$2"; shift 2;;
        -- ) shift; break ;;
        * ) break ;;
    esac
done

export ROS_IP=$LOCAL_IP
export ROS_MASTER_URI=http://"$MACHINE_IP":"$PORT"

roslaunch tony_local_planner gain_tuning.launch machine_ip:=$MACHINE_IP setup_script:=$SCRIPT_DIR global_planner:=$GLOBAL_PLANNER