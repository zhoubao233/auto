#!/usr/bin/env bash
set -u

# Record the complete mission_auto_avoid data chain.
# Stop with Ctrl-C; rosbag will close the bag cleanly.

source /opt/ros/noetic/setup.bash
source /home/coolfly/auto/kufei_auto/uav_indoor/devel/setup.bash
source /home/coolfly/auto/kufei_auto/Fast-Drone-250/devel/setup.bash

BAG_DIR="${HOME}/rosbags"
LABEL="${1:-mission_auto_avoid}"
STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p "${BAG_DIR}"

TOPICS=(
  /mavros/state
  /mavros/local_position/odom
  /mavros/local_position/pose
  /mavros/local_position/velocity_local
  /mavros/local_position/velocity_body
  /mavros/imu/data
  /ego/odom_world_velocity
  /mavros/rc/in
  /mavros/setpoint_raw/local
  /mavros/mission/waypoints
  /mavros/global_position/gp_origin
  /mavros/home_position/home
  /mavros/global_position/global
  /livox/lidar
  /world_cloud
  /drone_0_ego_planner_node/grid_map/occupancy
  /drone_0_ego_planner_node/grid_map/occupancy_inflate
  /ego_input_target
  /drone_0_planning/bspline
  /position_cmd
  /mission_auto_avoid/avoidance_active
  /auto_avoid_manager/closest_distance
  /auto_avoid_manager/route_closest_distance
  /auto_avoid_manager/local_closest_distance
  /auto_avoid_manager/route_distance
  /auto_avoid_manager/obstacle_ahead
  /auto_avoid_manager/planner_cmd_valid
  /auto_avoid_manager/auto_resume_ready
  /mode_select_flag
  /rosout
  /tf
  /tf_static
)

OUT="${BAG_DIR}/${LABEL}_${STAMP}"
echo "Recording to: ${OUT}_*.bag"
echo "Press Ctrl-C to stop."

rosbag record \
  --lz4 \
  --split --size=2048 \
  --buffsize=1024 \
  -O "${OUT}" \
  "${TOPICS[@]}"
