#!/usr/bin/env bash
# Source this file in every WSL terminal that interacts with the ROS 2 bridge.

_armpi_network_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_armpi_project_dir="$(cd "${_armpi_network_dir}/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "${_armpi_project_dir}/ros2_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI="file://${_armpi_project_dir}/network/cyclonedds_wsl.xml"
export ARMPI_PROTOCOL_PATH="${_armpi_project_dir}/protocol"

unset _armpi_network_dir
unset _armpi_project_dir
