#!/bin/bash

[[ $2 == "NO_ENV_VAR__ROBOT__" ]] && robot_name=$ROBOT || robot_name=$2

tuning="$1"
sim_dir="$(rospack find tony_local_planner)/robots/$robot_name"

RED='\033[0;31m'
ORANGE='\033[0;33m'
NC='\033[0m'

[[ "$(rosnode list | grep amcl)" == *"amcl"* ]] && have_amcl_node=true || have_amcl_node=false
[[ "$(rosnode list | grep map_server)" == *"map_server"* ]] && have_map_server_node=true || have_map_server_node=false

if [[ $have_amcl_node == true ]] || [[ $have_map_server_node == true ]]; then
    if [[ $tuning == false ]]; then
        >&2 echo -e "${RED}[ERROR] Existing amcl (or map_server) node detected, might be an indication" \
                    "that it is currently running on non-simulation environment." \
                    "Kill the nodes first if that is not the case.${NC}"
        exit 1
    fi

    exit 0
else    # no amcl and map server node
    if [[ $tuning == true ]]; then
      >&2 echo -e   "${ORANGE}[WARNING] No amcl and map_server node detected during tuning mode," \
                    "either tuning mode is launched in local PC, which may be pointless," \
                    "and still need additional setups (e.g. coordinate transform), or the" \
                    "original nodes died in the machine, which if is indeed the case, make sure you" \
                    "relaunch the amcl and map_server before gain tuning.${NC}"
    fi

    args="sim_dir:="$sim_dir""
    roslaunch --wait "$sim_dir/localization_sim.launch" $args
fi