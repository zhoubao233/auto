#!/usr/bin/env python3

import math
import os

import rospy
import sensor_msgs.point_cloud2 as pc2
import yaml
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition, State, Waypoint, WaypointList
from mavros_msgs.srv import SetMode, SetModeRequest, WaypointSetCurrent
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, UInt8
from std_srvs.srv import SetBool


class AutoAvoidManager:
    EARTH_RADIUS_M = 6378137.0
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_NAV_LAND = 21
    MAV_CMD_NAV_TAKEOFF = 22
    SUPPORTED_FCU_WAYPOINT_COMMANDS = {
        MAV_CMD_NAV_WAYPOINT,
        MAV_CMD_NAV_LAND,
        MAV_CMD_NAV_TAKEOFF,
    }
    SUPPORTED_FCU_RELATIVE_ALT_FRAMES = {
        Waypoint.FRAME_GLOBAL_REL_ALT,
        Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT,
    }
    SUPPORTED_FCU_ABSOLUTE_ALT_FRAMES = {
        Waypoint.FRAME_GLOBAL,
        Waypoint.FRAME_GLOBAL_INT,
        Waypoint.FRAME_GLOBAL_TERRAIN_ALT,
        Waypoint.FRAME_GLOBAL_TERRAIN_ALT_INT,
    }

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.point_cloud_topic = rospy.get_param("~point_cloud_topic", "/lidar_in_world")
        self.goal_topic = rospy.get_param("~goal_topic", "/ego_input_target")
        self.goal_frame_id = rospy.get_param("~goal_frame_id", "world")
        self.avoidance_flag_topic = rospy.get_param("~avoidance_flag_topic", "/mission_auto_avoid/avoidance_active")
        self.mode_select_flag_topic = rospy.get_param("~mode_select_flag_topic", "/mode_select_flag")
        self.planner_cmd_topic = rospy.get_param("~planner_cmd_topic", "/position_cmd")
        self.planning_service = rospy.get_param("~planning_service", "/drone_0_ego_planner_node/set_planning_enabled")
        self.planning_service_timeout = float(rospy.get_param("~planning_service_timeout", 5.0))
        self.waypoint_file = rospy.get_param("~waypoint_file", "")
        self.mission_ready_param = rospy.get_param("~mission_ready_param", "/mission_auto_avoid/mission_ready")
        self.mission_source = str(rospy.get_param("~mission_source", "fcu")).strip().lower()
        if self.mission_source not in ("file", "fcu"):
            rospy.logwarn("mission_auto_avoid: invalid mission_source=%s, fallback to file.", self.mission_source)
            self.mission_source = "file"

        self.fcu_waypoints_topic = rospy.get_param("~fcu_waypoints_topic", "/mavros/mission/waypoints")
        self.fcu_origin_topic = rospy.get_param("~fcu_origin_topic", "/mavros/global_position/gp_origin")
        self.fcu_home_topic = rospy.get_param("~fcu_home_topic", "/mavros/home_position/home")
        self.fcu_set_current_service = rospy.get_param("~fcu_set_current_service", "/mavros/mission/set_current")
        self.sync_fcu_current_on_auto_resume = rospy.get_param("~sync_fcu_current_on_auto_resume", False)
        self.fcu_set_current_dist_threshold = float(
            rospy.get_param("~fcu_set_current_dist_threshold", rospy.get_param("~waypoint_reached_radius", 1.0))
        )

        self.guided_mode = rospy.get_param("~guided_mode", "GUIDED")
        self.auto_mode = rospy.get_param("~auto_mode", "AUTO")
        self.return_mode_names = self._normalize_mode_names(
            rospy.get_param("~return_mode_names", "RTL,RTN,AUTO.RTL,AUTO.RTN")
        )
        self.set_auto_on_start = rospy.get_param("~set_auto_on_start", True)
        self.auto_start_delay = rospy.Duration(rospy.get_param("~auto_start_delay", 2.0))

        legacy_obstacle_distance_threshold = rospy.get_param("~obstacle_distance_threshold", 0.8)
        legacy_clear_distance_threshold = rospy.get_param("~clear_distance_threshold", 1.2)
        legacy_lookahead_distance = rospy.get_param("~lookahead_distance", 8.0)
        legacy_min_forward_distance = rospy.get_param("~min_forward_distance", 0.5)

        self.route_obstacle_distance_threshold = rospy.get_param(
            "~route_obstacle_distance_threshold",
            legacy_obstacle_distance_threshold,
        )
        self.route_clear_distance_threshold = rospy.get_param(
            "~route_clear_distance_threshold",
            legacy_clear_distance_threshold,
        )
        self.route_lookahead_distance = rospy.get_param(
            "~route_lookahead_distance",
            legacy_lookahead_distance,
        )
        self.route_min_forward_distance = rospy.get_param(
            "~route_min_forward_distance",
            legacy_min_forward_distance,
        )

        self.local_obstacle_distance_threshold = rospy.get_param(
            "~local_obstacle_distance_threshold",
            legacy_obstacle_distance_threshold,
        )
        self.local_clear_distance_threshold = rospy.get_param(
            "~local_clear_distance_threshold",
            legacy_clear_distance_threshold,
        )
        self.local_lookahead_distance = rospy.get_param(
            "~local_lookahead_distance",
            min(3.0, float(legacy_lookahead_distance)),
        )
        self.local_min_forward_distance = rospy.get_param(
            "~local_min_forward_distance",
            legacy_min_forward_distance,
        )
        self.waypoint_reached_radius = rospy.get_param("~waypoint_reached_radius", 1.0)
        self.waypoint_hold_time = rospy.Duration(max(0.0, float(rospy.get_param("~waypoint_hold_time", 3.0))))
        self.route_rejoin_tolerance = rospy.get_param("~route_rejoin_tolerance", 0.1)
        self.route_rejoin_enter_tolerance = rospy.get_param(
            "~route_rejoin_enter_tolerance",
            max(self.route_rejoin_tolerance + 0.1, self.route_rejoin_tolerance * 2.0),
        )
        self.avoidance_min_duration = rospy.Duration(rospy.get_param("~avoidance_min_duration", 2.0))
        self.clear_hold_time = rospy.Duration(rospy.get_param("~clear_hold_time", 1.0))
        self.guided_exit_debounce = rospy.Duration(rospy.get_param("~guided_exit_debounce", 2.0))
        self.planner_cmd_timeout = rospy.Duration(rospy.get_param("~planner_cmd_timeout", 0.5))
        self.planner_valid_hold_time = rospy.Duration(rospy.get_param("~planner_valid_hold_time", 1.0))
        self.planner_motion_recent_window = rospy.Duration(rospy.get_param("~planner_motion_recent_window", 1.0))
        self.guided_hold_speed_threshold = float(rospy.get_param("~guided_hold_speed_threshold", 0.2))
        self.planner_cmd_min_speed = rospy.get_param("~planner_cmd_min_speed", 0.15)
        self.planner_cmd_min_acc = rospy.get_param("~planner_cmd_min_acc", 0.2)
        self.planner_cmd_min_pos_error = rospy.get_param("~planner_cmd_min_pos_error", 0.2)
        self.require_new_planner_traj = rospy.get_param("~require_new_planner_traj", True)
        self.rejoin_forward_distance = rospy.get_param("~rejoin_forward_distance", 2.0)
        self.pre_takeover_goal_retry_interval = rospy.Duration(
            rospy.get_param("~pre_takeover_goal_retry_interval", 0.5)
        )
        self.stale_goal_retry_interval = rospy.Duration(
            rospy.get_param("~stale_goal_retry_interval", 1.0)
        )
        self.takeover_max_cmd_distance = rospy.get_param("~takeover_max_cmd_distance", 1.0)
        self.odom_timeout = rospy.Duration(rospy.get_param("~odom_timeout", 1.0))
        self.inflated_cloud_topic = rospy.get_param(
            "~inflated_cloud_topic",
            "/drone_0_ego_planner_node/grid_map/occupancy_inflate",
        )
        self.rejoin_path_lookahead_distance = rospy.get_param("~rejoin_path_lookahead_distance", 5.0)
        self.rejoin_clear_distance_threshold = rospy.get_param(
            "~rejoin_clear_distance_threshold",
            self.local_clear_distance_threshold,
        )
        self.inflated_rejoin_clear_distance_threshold = rospy.get_param(
            "~inflated_rejoin_clear_distance_threshold",
            0.25,
        )
        self.rejoin_current_clearance_threshold = rospy.get_param(
            "~rejoin_current_clearance_threshold",
            0.25,
        )
        self.exit_current_clearance_threshold = float(
            rospy.get_param(
                "~exit_current_clearance_threshold",
                max(float(self.rejoin_current_clearance_threshold), 0.5),
            )
        )

        self.state_msg = None
        self.odom_msg = None
        self.last_odom_time = rospy.Time(0)
        self.latest_points = []
        self.latest_inflated_points = []
        self.active_waypoint_index = 0
        self.avoidance_active = False
        self.avoidance_start_time = None
        self.clear_since = None
        self.last_goal_waypoint_index = None
        self.last_closest_distance = float("inf")
        self.last_route_closest_distance = float("inf")
        self.last_local_closest_distance = float("inf")
        self.last_obstacle_source = "none"
        self.last_mode_request_time = rospy.Time(0)
        self.auto_request_sent = False
        self.first_segment_start = None
        self.last_goal_target = None
        self.last_goal_kind = None
        self.waypoint_arrival_time = None
        self.last_planner_cmd_msg = None
        self.last_planner_cmd_time = rospy.Time(0)
        self.last_planner_traj_id = None
        self.last_planner_traj_change_time = None
        self.avoidance_entry_traj_id = None
        self.avoidance_new_traj_seen = False
        self.planner_valid_since = None
        self.planner_validated_for_exit = False
        self.last_planner_motion_time = None
        self.exit_ready_since = None
        self.planning_enabled_state = None
        self.rejoin_active = False
        self.avoidance_takeover_active = False
        self.avoidance_goal_released = False
        self.last_forced_goal_time = rospy.Time(0)
        self.resume_mode = None
        self.mode_select_flag = None
        self.fcu_origin = None
        self.fcu_home_position = None
        self.fcu_waypoint_list = None
        self.fcu_nav_waypoints = []
        self.fcu_current_nav_index = None
        self.fcu_current_target_seq = None
        self.fcu_mission_ready_state = None

        self.start_time = rospy.Time.now()
        self.waypoints = self._load_waypoints() if self.mission_source == "file" else []

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.avoidance_active_pub = rospy.Publisher(self.avoidance_flag_topic, Bool, queue_size=1, latch=True)
        self.closest_distance_pub = rospy.Publisher("~closest_distance", Float32, queue_size=1)
        self.route_closest_distance_pub = rospy.Publisher("~route_closest_distance", Float32, queue_size=1)
        self.local_closest_distance_pub = rospy.Publisher("~local_closest_distance", Float32, queue_size=1)
        self.route_distance_pub = rospy.Publisher("~route_distance", Float32, queue_size=1)
        self.obstacle_flag_pub = rospy.Publisher("~obstacle_ahead", Bool, queue_size=1)
        self.planner_cmd_valid_pub = rospy.Publisher("~planner_cmd_valid", Bool, queue_size=1)
        self.auto_resume_ready_pub = rospy.Publisher("~auto_resume_ready", Bool, queue_size=1)

        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.set_planning_client = rospy.ServiceProxy(self.planning_service, SetBool)
        self.set_current_client = None
        if self.mission_source == "fcu":
            self.set_current_client = rospy.ServiceProxy(self.fcu_set_current_service, WaypointSetCurrent)
            rospy.set_param(self.mission_ready_param, False)

        rospy.Subscriber("/mavros/state", State, self._state_callback, queue_size=10)
        rospy.Subscriber(self.mode_select_flag_topic, UInt8, self._mode_select_flag_callback, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=10)
        rospy.Subscriber(self.point_cloud_topic, PointCloud2, self._cloud_callback, queue_size=1)
        if self.inflated_cloud_topic:
            rospy.Subscriber(self.inflated_cloud_topic, PointCloud2, self._inflated_cloud_callback, queue_size=1)
        rospy.Subscriber(self.planner_cmd_topic, PositionCommand, self._planner_cmd_callback, queue_size=20)
        if self.mission_source == "fcu":
            rospy.Subscriber(self.fcu_origin_topic, GeoPointStamped, self._fcu_origin_callback, queue_size=1)
            rospy.Subscriber(self.fcu_home_topic, HomePosition, self._fcu_home_callback, queue_size=1)
            rospy.Subscriber(self.fcu_waypoints_topic, WaypointList, self._fcu_waypoint_list_callback, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self._timer_callback)
        rospy.on_shutdown(self._on_shutdown)
        self.avoidance_active_pub.publish(Bool(False))

        if self.mission_source == "fcu":
            rospy.loginfo(
                "mission_auto_avoid: using FCU mission source, waiting for %s, %s and %s (return modes use FCU home).",
                self.fcu_origin_topic,
                self.fcu_waypoints_topic,
                self.fcu_home_topic,
            )
        else:
            rospy.loginfo("mission_auto_avoid: loaded %d mission waypoints.", len(self.waypoints))

    def _load_waypoints(self):
        if self.waypoint_file:
            file_waypoints = self._load_waypoints_from_file(self.waypoint_file)
            if file_waypoints:
                return file_waypoints
            rospy.logwarn("mission_auto_avoid: fallback to launch waypoint params because waypoint file is unavailable or empty.")

        waypoints = []
        count = int(rospy.get_param("~mission/point_num", 0))
        for idx in range(count):
            x = rospy.get_param("~mission/point{}_x".format(idx), None)
            y = rospy.get_param("~mission/point{}_y".format(idx), None)
            z = rospy.get_param("~mission/point{}_z".format(idx), None)
            if x is None or y is None or z is None:
                continue
            waypoints.append((float(x), float(y), float(z)))
        return waypoints

    def _load_waypoints_from_file(self, path):
        if not os.path.isfile(path):
            rospy.logwarn("mission_auto_avoid: waypoint file not found: %s", path)
            return []

        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:
            rospy.logwarn("mission_auto_avoid: failed to read waypoint file %s: %s", path, exc)
            return []

        mission_cfg = data.get("mission", data) if isinstance(data, dict) else {}
        if not isinstance(mission_cfg, dict):
            rospy.logwarn("mission_auto_avoid: waypoint file format is invalid: %s", path)
            return []

        file_frame_id = mission_cfg.get("frame_id")
        if isinstance(file_frame_id, str) and file_frame_id:
            self.goal_frame_id = file_frame_id

        file_radius = mission_cfg.get("waypoint_reached_radius")
        if isinstance(file_radius, (int, float)) and file_radius > 0.0:
            self.waypoint_reached_radius = float(file_radius)

        raw_waypoints = mission_cfg.get("waypoints", [])
        if not isinstance(raw_waypoints, list):
            rospy.logwarn("mission_auto_avoid: mission.waypoints must be a list in %s", path)
            return []

        waypoints = []
        for idx, item in enumerate(raw_waypoints):
            waypoint = self._parse_waypoint_item(item, idx, path)
            if waypoint is not None:
                waypoints.append(waypoint)

        rospy.loginfo("mission_auto_avoid: loaded %d waypoints from %s", len(waypoints), path)
        return waypoints

    def _parse_waypoint_item(self, item, idx, path):
        if isinstance(item, dict):
            x = item.get("x")
            y = item.get("y")
            z = item.get("z")
        elif isinstance(item, (list, tuple)) and len(item) >= 3:
            x, y, z = item[0], item[1], item[2]
        else:
            rospy.logwarn("mission_auto_avoid: invalid waypoint #%d in %s", idx, path)
            return None

        try:
            return (float(x), float(y), float(z))
        except (TypeError, ValueError):
            rospy.logwarn("mission_auto_avoid: waypoint #%d has non-numeric coordinates in %s", idx, path)
            return None

    def _state_callback(self, msg):
        self.state_msg = msg

    def _mode_select_flag_callback(self, msg):
        self.mode_select_flag = int(msg.data)

    def _odom_callback(self, msg):
        self.odom_msg = msg
        self.last_odom_time = rospy.Time.now()

    def _fcu_origin_callback(self, msg):
        self.fcu_origin = (
            float(msg.position.latitude),
            float(msg.position.longitude),
            float(msg.position.altitude),
        )
        self._refresh_fcu_nav_waypoints()

    def _fcu_home_callback(self, msg):
        self.fcu_home_position = (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        )

    def _fcu_waypoint_list_callback(self, msg):
        self.fcu_waypoint_list = msg
        self._refresh_fcu_nav_waypoints()

    @staticmethod
    def _read_cloud_points(msg):
        points = []
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append((float(x), float(y), float(z)))
        return points

    def _cloud_callback(self, msg):
        points = self._read_cloud_points(msg)
        self.latest_points = points

    def _inflated_cloud_callback(self, msg):
        points = self._read_cloud_points(msg)
        self.latest_inflated_points = points

    def _planner_cmd_callback(self, msg):
        now = rospy.Time.now()
        traj_id = int(msg.trajectory_id)
        if self.last_planner_traj_id is None or traj_id != self.last_planner_traj_id:
            self.last_planner_traj_change_time = now

        self.last_planner_cmd_msg = msg
        self.last_planner_cmd_time = now
        self.last_planner_traj_id = traj_id

        if not self.avoidance_active or self.avoidance_new_traj_seen:
            return

        if not self.require_new_planner_traj:
            self.avoidance_new_traj_seen = True
            return

        if self.avoidance_entry_traj_id is None or self.last_planner_traj_id != self.avoidance_entry_traj_id:
            self.avoidance_new_traj_seen = True
            rospy.loginfo(
                "mission_auto_avoid: planner produced a new avoidance trajectory (entry_traj_id=%s, current_traj_id=%d).",
                str(self.avoidance_entry_traj_id),
                self.last_planner_traj_id,
            )

    def _refresh_fcu_nav_waypoints(self):
        if self.mission_source != "fcu":
            return

        nav_waypoints = []
        if self.fcu_origin is not None and self.fcu_waypoint_list is not None:
            for seq, waypoint in enumerate(self.fcu_waypoint_list.waypoints):
                point = self._fcu_waypoint_to_local(waypoint)
                if point is None:
                    continue
                nav_waypoints.append(
                    {
                        "seq": seq,
                        "point": point,
                        "command": int(waypoint.command),
                    }
                )

        self.fcu_nav_waypoints = nav_waypoints
        self._update_fcu_mission_ready_state()

    def _update_fcu_mission_ready_state(self):
        if self.mission_source != "fcu":
            return

        ready = self._fcu_mission_ready()
        if self.fcu_mission_ready_state == ready:
            return

        self.fcu_mission_ready_state = ready
        rospy.set_param(self.mission_ready_param, ready)
        if ready:
            rospy.loginfo(
                "mission_auto_avoid: FCU mission ready with %d supported waypoints.",
                len(self.fcu_nav_waypoints),
            )
        else:
            rospy.logwarn(
                "mission_auto_avoid: FCU mission is not ready yet (origin=%s waypoint_list=%s supported=%d).",
                "yes" if self.fcu_origin is not None else "no",
                "yes" if self.fcu_waypoint_list is not None else "no",
                len(self.fcu_nav_waypoints),
            )

    def _fcu_mission_ready(self):
        return self.fcu_origin is not None and self.fcu_waypoint_list is not None and bool(self.fcu_nav_waypoints)

    def _fcu_waypoint_to_local(self, waypoint):
        if int(waypoint.command) not in self.SUPPORTED_FCU_WAYPOINT_COMMANDS:
            return None

        if waypoint.frame in self.SUPPORTED_FCU_RELATIVE_ALT_FRAMES:
            altitude = float(waypoint.z_alt)
        elif waypoint.frame in self.SUPPORTED_FCU_ABSOLUTE_ALT_FRAMES:
            altitude = float(waypoint.z_alt) - float(self.fcu_origin[2])
        else:
            return None

        latitude = float(waypoint.x_lat)
        longitude = float(waypoint.y_long)
        if not (math.isfinite(latitude) and math.isfinite(longitude) and math.isfinite(altitude)):
            return None

        east_m, north_m = self._geodetic_to_enu(latitude, longitude)
        return (east_m, north_m, altitude)

    def _geodetic_to_enu(self, latitude_deg, longitude_deg):
        lat0_deg, lon0_deg, _alt0 = self.fcu_origin
        lat0_rad = math.radians(lat0_deg)
        east_m = (
            math.radians(longitude_deg - lon0_deg)
            * self.EARTH_RADIUS_M
            * max(math.cos(lat0_rad), 1e-9)
        )
        north_m = math.radians(latitude_deg - lat0_deg) * self.EARTH_RADIUS_M
        return east_m, north_m

    def _timer_callback(self, _event):
        if self.state_msg is None or self.odom_msg is None:
            return

        now = rospy.Time.now()
        if not self._guided_requests_allowed():
            self.planner_cmd_valid_pub.publish(Bool(False))
            self.auto_resume_ready_pub.publish(Bool(False))
            if self.avoidance_active:
                self._cancel_avoidance_for_loiter()
            self._set_planning_enabled(False, timeout=0.2)
            return

        if now - self.last_odom_time > self.odom_timeout:
            self.planner_cmd_valid_pub.publish(Bool(False))
            self.auto_resume_ready_pub.publish(Bool(False))
            if self.avoidance_active:
                rospy.logwarn_throttle(
                    1.0,
                    "mission_auto_avoid: odom is stale for %.2fs, hold avoidance state and do not resume AUTO.",
                    (now - self.last_odom_time).to_sec(),
                )
            self._set_planning_enabled(False, timeout=0.2)
            return

        if not self.state_msg.connected:
            self.auto_resume_ready_pub.publish(Bool(False))
            if self.avoidance_active:
                rospy.logwarn_throttle(
                    1.0,
                    "mission_auto_avoid: FCU is disconnected, cannot switch modes or finish avoidance.",
                )
            self._set_planning_enabled(False, timeout=0.2)
            return

        self._sync_planning_enabled()
        self._try_set_auto_on_start()

        target = self._current_target_waypoint()
        if target is None:
            self.closest_distance_pub.publish(Float32(float("inf")))
            self.route_closest_distance_pub.publish(Float32(float("inf")))
            self.local_closest_distance_pub.publish(Float32(float("inf")))
            self.route_distance_pub.publish(Float32(0.0))
            self.obstacle_flag_pub.publish(Bool(False))
            if self.avoidance_active:
                self._exit_avoidance("mission_finished")
            self.rejoin_active = False
            return

        route_start, route_end = self._current_route_segment(target)
        route_distance, rejoin_target = self._route_distance_and_rejoin_target(route_start, route_end)
        route_rejoin_needed = False

        route_closest_distance, _route_closest_point = self._closest_obstacle_distance_to_segment(
            route_start,
            route_end,
            self.route_lookahead_distance,
            self.route_min_forward_distance,
            reference_point=self._odom_position(),
        )
        local_closest_distance, _local_closest_point = self._closest_obstacle_distance_to_target_line(target)
        closest_distance = min(route_closest_distance, local_closest_distance)

        self.last_closest_distance = closest_distance
        self.last_route_closest_distance = route_closest_distance
        self.last_local_closest_distance = local_closest_distance
        self.closest_distance_pub.publish(Float32(closest_distance))
        self.route_closest_distance_pub.publish(Float32(route_closest_distance))
        self.local_closest_distance_pub.publish(Float32(local_closest_distance))
        self.route_distance_pub.publish(Float32(route_distance))

        route_obstacle_ahead = route_closest_distance <= self.route_obstacle_distance_threshold
        local_obstacle_ahead = local_closest_distance <= self.local_obstacle_distance_threshold
        obstacle_ahead = route_obstacle_ahead or local_obstacle_ahead

        route_clear_ahead = (
            math.isinf(route_closest_distance)
            or route_closest_distance >= self.route_clear_distance_threshold
        )
        local_clear_ahead = (
            math.isinf(local_closest_distance)
            or local_closest_distance >= self.local_clear_distance_threshold
        )
        clear_ahead = route_clear_ahead and local_clear_ahead
        self.last_obstacle_source = self._obstacle_source(route_obstacle_ahead, local_obstacle_ahead)
        self.obstacle_flag_pub.publish(Bool(obstacle_ahead))

        if not self.avoidance_active:
            self.planner_cmd_valid_pub.publish(Bool(False))
            self.auto_resume_ready_pub.publish(Bool(False))
            if obstacle_ahead:
                self._enter_avoidance(target)
            return

        self._request_mode(self.guided_mode)

        planner_cmd_valid = self._update_planner_validation()
        self.planner_cmd_valid_pub.publish(Bool(planner_cmd_valid))
        exit_current_clearance, exit_current_clearance_source = self._exit_current_clearance()
        exit_current_clearance_safe = (
            math.isinf(exit_current_clearance)
            or exit_current_clearance >= self.exit_current_clearance_threshold
        )

        if not self.avoidance_takeover_active:
            self.clear_since = None
            self.exit_ready_since = None
            self.rejoin_active = False
            self.auto_resume_ready_pub.publish(Bool(False))
            if not self._guided_hold_ready_for_goal_release():
                return
            self._publish_goal(
                target,
                goal_kind="waypoint",
                force=(
                    (not self.avoidance_goal_released)
                    or self._should_force_goal_retry(self.pre_takeover_goal_retry_interval)
                ),
            )
            self.avoidance_goal_released = True
            if self._planner_takeover_ready():
                self._activate_avoidance_takeover("new_avoidance_trajectory_ready")
            else:
                rospy.loginfo_throttle(
                    1.0,
                    "mission_auto_avoid: waiting for a fresh planner trajectory before GUIDED takeover.",
                )
            return

        if obstacle_ahead:
            self.clear_since = None
            self.exit_ready_since = None
            self.auto_resume_ready_pub.publish(Bool(False))
            self.rejoin_active = False
            self._publish_goal(
                target,
                goal_kind="waypoint",
                force=(not planner_cmd_valid and self._should_force_goal_retry(self.stale_goal_retry_interval)),
            )
            return

        if clear_ahead:
            if self.clear_since is None:
                self.clear_since = rospy.Time.now()
        else:
            self.clear_since = None
            self.exit_ready_since = None
            self.auto_resume_ready_pub.publish(Bool(False))
            self.rejoin_active = False
            self._publish_goal(target, goal_kind="waypoint")
            return

        clear_hold_ready = (
            self.clear_since is not None
            and rospy.Time.now() - self.clear_since >= self.clear_hold_time
        )
        if not clear_hold_ready:
            self.exit_ready_since = None
            self.auto_resume_ready_pub.publish(Bool(False))
            self.rejoin_active = False
            self._publish_goal(target, goal_kind="waypoint")
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep avoiding until the path has stayed clear long enough.",
            )
            return

        route_rejoin_needed = self._update_rejoin_state(route_distance)

        if route_rejoin_needed:
            self.exit_ready_since = None
            self.auto_resume_ready_pub.publish(Bool(False))
            rejoin_clear, rejoin_line_distance, rejoin_current_clearance, rejoin_source, rejoin_threshold = (
                self._rejoin_path_clear(rejoin_target)
            )
            if not rejoin_clear:
                self._publish_goal(
                    target,
                    goal_kind="waypoint",
                    force=(not planner_cmd_valid and self._should_force_goal_retry(self.stale_goal_retry_interval)),
                )
                rospy.loginfo_throttle(
                    1.0,
                    "mission_auto_avoid: keep avoiding before route rejoin; %s rejoin path not clear "
                    "(line=%.3f/%.3f, current=%.3f, target=(%.2f, %.2f, %.2f)).",
                    rejoin_source,
                    rejoin_line_distance,
                    rejoin_threshold,
                    rejoin_current_clearance,
                    rejoin_target[0],
                    rejoin_target[1],
                    rejoin_target[2],
                )
                return

            self._publish_goal(
                rejoin_target,
                goal_kind="route_rejoin",
                force=(not planner_cmd_valid and self._should_force_goal_retry(self.stale_goal_retry_interval)),
            )
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: route rejoin active, target=(%.2f, %.2f, %.2f), route_distance=%.3f, planner_valid=%s.",
                rejoin_target[0],
                rejoin_target[1],
                rejoin_target[2],
                route_distance,
                "true" if planner_cmd_valid else "false",
            )
            return

        self._publish_goal(target, goal_kind="waypoint")
        auto_resume_ready = (
            clear_ahead
            and planner_cmd_valid
            and self.planner_validated_for_exit
            and exit_current_clearance_safe
            and rospy.Time.now() - self.avoidance_start_time >= self.avoidance_min_duration
            and self.clear_since is not None
            and rospy.Time.now() - self.clear_since >= self.clear_hold_time
        )
        if auto_resume_ready:
            if self.exit_ready_since is None:
                self.exit_ready_since = rospy.Time.now()
            self.auto_resume_ready_pub.publish(Bool(True))
            if rospy.Time.now() - self.exit_ready_since >= self.guided_exit_debounce:
                self._exit_avoidance("path_clear_route_rejoined_planner_valid")
        else:
            self.exit_ready_since = None
            self.auto_resume_ready_pub.publish(Bool(False))
            self._log_exit_block_reason(
                clear_ahead,
                route_rejoin_needed,
                planner_cmd_valid,
                exit_current_clearance_safe,
                exit_current_clearance,
                exit_current_clearance_source,
            )

    def _try_set_auto_on_start(self):
        if not self.set_auto_on_start or self.auto_request_sent:
            return
        if rospy.Time.now() - self.start_time < self.auto_start_delay:
            return
        if not self.state_msg.connected:
            return
        if not self._mission_ready():
            return
        if self._request_mode(self.auto_mode):
            self.auto_request_sent = True

    def _current_target_waypoint(self):
        if self.mission_source == "fcu":
            return self._current_target_waypoint_fcu()
        return self._current_target_waypoint_file()

    def _current_target_waypoint_file(self):
        if not self.waypoints:
            return None

        current_pos = self._odom_position()
        if self.first_segment_start is None:
            self.first_segment_start = current_pos
        while self.active_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.active_waypoint_index]
            if self._distance(current_pos, target) > self.waypoint_reached_radius:
                self.waypoint_arrival_time = None
                return target

            now = rospy.Time.now()
            if self.waypoint_arrival_time is None:
                self.waypoint_arrival_time = now

            if now - self.waypoint_arrival_time < self.waypoint_hold_time:
                return target

            reached_index = self.active_waypoint_index
            self.active_waypoint_index += 1
            self.last_goal_waypoint_index = None
            self.waypoint_arrival_time = None
            if self.active_waypoint_index < len(self.waypoints):
                rospy.loginfo(
                    "mission_auto_avoid: reached mission waypoint index %d, next target index %d.",
                    reached_index,
                    self.active_waypoint_index,
                )
            else:
                rospy.loginfo(
                    "mission_auto_avoid: reached final mission waypoint index %d.",
                    reached_index,
                )

        return None

    def _current_target_waypoint_fcu(self):
        self.fcu_current_nav_index = None
        self.fcu_current_target_seq = None

        if self._is_return_mode(self.state_msg.mode if self.state_msg is not None else None):
            return self._current_return_target_waypoint_fcu()

        if not self._fcu_mission_ready():
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: waiting for FCU mission data before obstacle management.",
            )
            return None

        current_seq = int(self.fcu_waypoint_list.current_seq)
        for idx, waypoint in enumerate(self.fcu_nav_waypoints):
            if waypoint["seq"] < current_seq:
                continue
            self.fcu_current_nav_index = idx
            self.fcu_current_target_seq = waypoint["seq"]
            return waypoint["point"]

        return None

    def _current_route_segment(self, target):
        if self.mission_source == "fcu":
            return self._current_route_segment_fcu(target)
        return self._current_route_segment_file(target)

    def _current_route_segment_file(self, target):
        if self.active_waypoint_index <= 0:
            if self.first_segment_start is None:
                self.first_segment_start = self._odom_position()
            return self.first_segment_start, target

        return self.waypoints[self.active_waypoint_index - 1], target

    def _current_route_segment_fcu(self, target):
        if self._is_return_mode(self.state_msg.mode if self.state_msg is not None else None):
            current_pos = self._odom_position()
            return current_pos, target

        if self.fcu_current_nav_index is None or not self.fcu_nav_waypoints:
            current_pos = self._odom_position()
            return current_pos, target

        if self.fcu_current_nav_index <= 0:
            return self._odom_position(), target

        return self.fcu_nav_waypoints[self.fcu_current_nav_index - 1]["point"], target

    def _route_distance_and_rejoin_target(self, segment_start, segment_end):
        current_pos = self._odom_position()
        seg_vec = (
            segment_end[0] - segment_start[0],
            segment_end[1] - segment_start[1],
            segment_end[2] - segment_start[2],
        )
        seg_len_sq = (
            seg_vec[0] * seg_vec[0]
            + seg_vec[1] * seg_vec[1]
            + seg_vec[2] * seg_vec[2]
        )

        if seg_len_sq < 1e-6:
            return self._distance(current_pos, segment_end), segment_end

        rel = (
            current_pos[0] - segment_start[0],
            current_pos[1] - segment_start[1],
            current_pos[2] - segment_start[2],
        )
        projection = (
            rel[0] * seg_vec[0]
            + rel[1] * seg_vec[1]
            + rel[2] * seg_vec[2]
        ) / seg_len_sq
        projection = min(1.0, max(0.0, projection))

        closest_route_point = (
            segment_start[0] + projection * seg_vec[0],
            segment_start[1] + projection * seg_vec[1],
            segment_start[2] + projection * seg_vec[2],
        )

        forward_projection = min(
            1.0,
            projection + max(0.0, float(self.rejoin_forward_distance)) / math.sqrt(seg_len_sq),
        )
        rejoin_target = (
            segment_start[0] + forward_projection * seg_vec[0],
            segment_start[1] + forward_projection * seg_vec[1],
            segment_start[2] + forward_projection * seg_vec[2],
        )
        return self._distance(current_pos, closest_route_point), rejoin_target

    def _closest_obstacle_distance_to_target_line(self, target):
        current_pos = self._odom_position()
        return self._closest_obstacle_distance_to_segment(
            current_pos,
            target,
            self.local_lookahead_distance,
            self.local_min_forward_distance,
        )

    def _closest_obstacle_distance_to_segment(
        self,
        segment_start,
        segment_end,
        lookahead_distance,
        min_forward_distance,
        reference_point=None,
        points=None,
    ):
        seg_vec = (
            segment_end[0] - segment_start[0],
            segment_end[1] - segment_start[1],
            segment_end[2] - segment_start[2],
        )
        seg_len = self._norm(seg_vec)
        if seg_len < 1e-6:
            return float("inf"), None

        dir_vec = (seg_vec[0] / seg_len, seg_vec[1] / seg_len, seg_vec[2] / seg_len)
        base_projection = 0.0
        if reference_point is not None:
            ref_rel = (
                reference_point[0] - segment_start[0],
                reference_point[1] - segment_start[1],
                reference_point[2] - segment_start[2],
            )
            base_projection = (
                ref_rel[0] * dir_vec[0]
                + ref_rel[1] * dir_vec[1]
                + ref_rel[2] * dir_vec[2]
            )
            base_projection = min(seg_len, max(0.0, base_projection))

        max_forward_distance = min(float(lookahead_distance), seg_len - base_projection)
        if max_forward_distance < float(min_forward_distance):
            return float("inf"), None

        min_dist = float("inf")
        min_point = None
        obstacle_points = self.latest_points if points is None else points
        for point in obstacle_points:
            rel = (
                point[0] - segment_start[0],
                point[1] - segment_start[1],
                point[2] - segment_start[2],
            )
            projection = rel[0] * dir_vec[0] + rel[1] * dir_vec[1] + rel[2] * dir_vec[2]
            if projection < 0.0 or projection > seg_len:
                continue

            forward_projection = projection - base_projection
            if forward_projection < min_forward_distance or forward_projection > max_forward_distance:
                continue

            lateral = (
                rel[0] - projection * dir_vec[0],
                rel[1] - projection * dir_vec[1],
                rel[2] - projection * dir_vec[2],
            )
            lateral_dist = self._norm(lateral)
            if lateral_dist < min_dist:
                min_dist = lateral_dist
                min_point = point

        return min_dist, min_point

    def _closest_distance_to_points(self, reference_point, points):
        min_dist = float("inf")
        for point in points:
            dist = self._distance(reference_point, point)
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def _rejoin_path_clear(self, rejoin_target):
        current_pos = self._odom_position()
        if self.latest_inflated_points:
            points = self.latest_inflated_points
            source = "inflated"
            line_threshold = self.inflated_rejoin_clear_distance_threshold
            current_threshold = self.rejoin_current_clearance_threshold
        else:
            points = self.latest_points
            source = "raw"
            line_threshold = self.rejoin_clear_distance_threshold
            current_threshold = 0.0

        line_distance, _closest_point = self._closest_obstacle_distance_to_segment(
            current_pos,
            rejoin_target,
            self.rejoin_path_lookahead_distance,
            self.local_min_forward_distance,
            points=points,
        )
        current_clearance = self._closest_distance_to_points(current_pos, points)

        line_clear = math.isinf(line_distance) or line_distance >= line_threshold
        current_clear = (
            current_threshold <= 0.0
            or math.isinf(current_clearance)
            or current_clearance >= current_threshold
        )
        return line_clear and current_clear, line_distance, current_clearance, source, line_threshold

    def _exit_current_clearance(self):
        if not self.latest_inflated_points:
            return float("inf"), "none"

        current_pos = self._odom_position()
        return self._closest_distance_to_points(current_pos, self.latest_inflated_points), "inflated"

    def _update_planner_validation(self):
        now = rospy.Time.now()
        if self.last_planner_cmd_msg is None:
            self.planner_valid_since = None
            return False

        cmd = self.last_planner_cmd_msg
        cmd_fresh = now - self.last_planner_cmd_time <= self.planner_cmd_timeout
        cmd_ready = cmd.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_READY
        traj_ready = self.avoidance_new_traj_seen or (not self.require_new_planner_traj)
        moving_cmd = self._planner_cmd_has_motion(cmd, require_recent_progress=True)

        if cmd_fresh and cmd_ready and traj_ready and moving_cmd:
            self.last_planner_motion_time = now
            if self.planner_valid_since is None:
                self.planner_valid_since = now
            elif (
                not self.planner_validated_for_exit
                and now - self.planner_valid_since >= self.planner_valid_hold_time
            ):
                self.planner_validated_for_exit = True
                rospy.loginfo(
                    "mission_auto_avoid: planner output stayed valid for %.2f s in current avoidance session.",
                    self.planner_valid_hold_time.to_sec(),
                )
        else:
            self.planner_valid_since = None

        if not self.planner_validated_for_exit:
            return False

        if not cmd_fresh or not cmd_ready:
            return False

        if self.last_planner_motion_time is None:
            return False

        return now - self.last_planner_motion_time <= self.planner_motion_recent_window

    def _planner_cmd_has_motion(self, cmd, require_recent_progress=False):
        current_pos = self._odom_position()
        cmd_pos = (cmd.position.x, cmd.position.y, cmd.position.z)
        cmd_vel = (cmd.velocity.x, cmd.velocity.y, cmd.velocity.z)
        cmd_acc = (cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z)

        if self._norm(cmd_vel) >= self.planner_cmd_min_speed or self._norm(cmd_acc) >= self.planner_cmd_min_acc:
            return True

        if self._distance(current_pos, cmd_pos) < self.planner_cmd_min_pos_error:
            return False

        if not require_recent_progress:
            return True

        if self.last_planner_traj_change_time is None:
            return False

        return rospy.Time.now() - self.last_planner_traj_change_time <= self.planner_motion_recent_window

    def _log_exit_block_reason(
        self,
        clear_ahead,
        route_rejoin_needed,
        planner_cmd_valid,
        exit_current_clearance_safe,
        exit_current_clearance,
        exit_current_clearance_source,
    ):
        if not clear_ahead:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep GUIDED because path is not clear yet (source=%s, route=%.3f/%.3f, local=%.3f/%.3f).",
                self.last_obstacle_source,
                self.last_route_closest_distance,
                self.route_clear_distance_threshold,
                self.last_local_closest_distance,
                self.local_clear_distance_threshold,
            )
            return

        if not exit_current_clearance_safe:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep GUIDED because current %s clearance is too small for AUTO resume (current=%.3f/%.3f).",
                exit_current_clearance_source,
                exit_current_clearance,
                self.exit_current_clearance_threshold,
            )
            return

        if route_rejoin_needed:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep GUIDED because route is not rejoined yet.",
            )
            return

        if not self.planner_validated_for_exit:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep GUIDED because planner has not yet sustained a valid avoidance trajectory in this session.",
            )
            return

        if not planner_cmd_valid:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep GUIDED because planner output is stale or hovering near the current position.",
            )
            return

        if self.clear_since is not None and rospy.Time.now() - self.clear_since < self.clear_hold_time:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep GUIDED because clear hold time is still accumulating.",
            )
            return

        if rospy.Time.now() - self.avoidance_start_time < self.avoidance_min_duration:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep GUIDED because minimum avoidance duration is still accumulating.",
            )
            return

        if self.exit_ready_since is not None and rospy.Time.now() - self.exit_ready_since < self.guided_exit_debounce:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: keep GUIDED because exit debounce is still accumulating.",
            )

    def _enter_avoidance(self, target):
        if not self._guided_requests_allowed():
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: skip avoidance because /mode_select_flag=%s keeps LOITER/manual control.",
                str(self.mode_select_flag),
            )
            return

        self.avoidance_active = True
        self.avoidance_takeover_active = False
        self.avoidance_goal_released = False
        self.avoidance_start_time = rospy.Time.now()
        self.resume_mode = self._mode_to_resume_after_avoidance()
        self.clear_since = None
        self.last_goal_waypoint_index = self._current_mission_progress_index()
        self.avoidance_entry_traj_id = self.last_planner_traj_id
        self.avoidance_new_traj_seen = False
        self.planner_valid_since = None
        self.planner_validated_for_exit = False
        self.last_planner_motion_time = None
        self.exit_ready_since = None
        self.last_forced_goal_time = rospy.Time.now()

        self.avoidance_active_pub.publish(Bool(True))
        self._request_mode(self.guided_mode)

        if not self._set_planning_enabled(True, timeout=self.planning_service_timeout):
            rospy.logwarn("mission_auto_avoid: failed to enable planner after switching to %s hold, will retry.", self.guided_mode)

        rospy.logwarn(
            "mission_auto_avoid: obstacle ahead, switch to %s hold, wait for speed <= %.2fm/s, then release planner goal. source=%s closest=%.3f route=%.3f local=%.3f target=(%.2f, %.2f, %.2f)",
            self.guided_mode,
            self.guided_hold_speed_threshold,
            self.last_obstacle_source,
            self.last_closest_distance,
            self.last_route_closest_distance,
            self.last_local_closest_distance,
            target[0],
            target[1],
            target[2],
        )

    def _exit_avoidance(self, reason):
        resume_mode = self.resume_mode or self.auto_mode
        self.avoidance_active_pub.publish(Bool(False))
        if resume_mode == self.auto_mode:
            self._maybe_sync_fcu_current_before_auto_resume()
        self._request_mode(resume_mode)
        self._reset_avoidance_state()

        rospy.logwarn("mission_auto_avoid: avoidance finished, switch back to %s (%s).", resume_mode, reason)

    def _guided_requests_allowed(self):
        return self.mode_select_flag is not None and self.mode_select_flag != 0

    def _reset_avoidance_state(self):
        self.avoidance_active = False
        self.avoidance_start_time = None
        self.clear_since = None
        self.last_goal_waypoint_index = None
        self.last_goal_target = None
        self.last_goal_kind = None
        self.avoidance_entry_traj_id = None
        self.avoidance_new_traj_seen = False
        self.planner_valid_since = None
        self.planner_validated_for_exit = False
        self.last_planner_motion_time = None
        self.exit_ready_since = None
        self.rejoin_active = False
        self.avoidance_takeover_active = False
        self.avoidance_goal_released = False
        self.last_forced_goal_time = rospy.Time(0)
        self.resume_mode = None

    def _cancel_avoidance_for_loiter(self):
        self.avoidance_active_pub.publish(Bool(False))
        self._reset_avoidance_state()
        rospy.logwarn_throttle(
            1.0,
            "mission_auto_avoid: /mode_select_flag=0, cancel avoidance state and block %s requests.",
            self.guided_mode,
        )

    def _guided_hold_ready_for_goal_release(self):
        if self.state_msg.mode != self.guided_mode:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: waiting for FCU to enter %s before releasing planner goal (current mode=%s).",
                self.guided_mode,
                self.state_msg.mode,
            )
            return False

        if self.guided_hold_speed_threshold <= 0.0:
            return True

        current_speed = self._odom_speed()
        if current_speed > self.guided_hold_speed_threshold:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: holding planner goal until %s speed drops below %.2fm/s (current %.2fm/s).",
                self.guided_mode,
                self.guided_hold_speed_threshold,
                current_speed,
            )
            return False

        return True

    def _planner_takeover_ready(self):
        if self.last_planner_cmd_msg is None:
            return False

        now = rospy.Time.now()
        cmd = self.last_planner_cmd_msg
        cmd_fresh = now - self.last_planner_cmd_time <= self.planner_cmd_timeout
        cmd_ready = cmd.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_READY
        traj_ready = self.avoidance_new_traj_seen or (not self.require_new_planner_traj)
        if not (cmd_fresh and cmd_ready and traj_ready and self._planner_cmd_has_motion(cmd)):
            return False

        if self.takeover_max_cmd_distance > 0.0:
            current_pos = self._odom_position()
            cmd_pos = (cmd.position.x, cmd.position.y, cmd.position.z)
            cmd_distance = self._distance(current_pos, cmd_pos)
            if cmd_distance > self.takeover_max_cmd_distance:
                rospy.logwarn_throttle(
                    1.0,
                    "mission_auto_avoid: waiting because first planner cmd is %.2fm from odom (limit %.2fm, traj=%d).",
                    cmd_distance,
                    self.takeover_max_cmd_distance,
                    int(cmd.trajectory_id),
                )
                return False

        return True

    def _activate_avoidance_takeover(self, reason):
        self.avoidance_takeover_active = True
        self.avoidance_active_pub.publish(Bool(True))
        self._request_mode(self.guided_mode)
        rospy.logwarn("mission_auto_avoid: planner ready, allow %s trajectory tracking (%s).", self.guided_mode, reason)

    def _publish_goal(self, target, goal_kind, force=False):
        if not force and self.last_goal_kind == goal_kind and self._same_point(self.last_goal_target, target):
            return

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.goal_frame_id
        msg.pose.position.x = target[0]
        msg.pose.position.y = target[1]
        msg.pose.position.z = target[2]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.last_goal_target = target
        self.last_goal_kind = goal_kind
        rospy.loginfo(
            "mission_auto_avoid: published %s goal target=(%.2f, %.2f, %.2f), force=%s.",
            goal_kind,
            target[0],
            target[1],
            target[2],
            "true" if force else "false",
        )

    def _request_mode(self, mode_name):
        if self.state_msg is None:
            return False
        if mode_name == self.guided_mode and not self._guided_requests_allowed():
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: skip mode request %s because /mode_select_flag=%s keeps LOITER/manual control.",
                mode_name,
                str(self.mode_select_flag),
            )
            return False
        if self.state_msg.mode == mode_name:
            return True

        now = rospy.Time.now()
        if now - self.last_mode_request_time < rospy.Duration(1.0):
            return False

        req = SetModeRequest()
        req.base_mode = 0
        req.custom_mode = mode_name

        try:
            response = self.set_mode_client(req)
        except rospy.ServiceException as exc:
            rospy.logwarn("mission_auto_avoid: set_mode(%s) failed: %s", mode_name, exc)
            self.last_mode_request_time = now
            return False

        self.last_mode_request_time = now
        if response.mode_sent:
            rospy.logwarn("mission_auto_avoid: requested mode %s.", mode_name)
            return True

        rospy.logwarn("mission_auto_avoid: FCU rejected mode %s.", mode_name)
        return False

    def _should_force_goal_retry(self, interval):
        now = rospy.Time.now()
        if now - self.last_forced_goal_time < interval:
            return False
        self.last_forced_goal_time = now
        return True

    def _mission_ready(self):
        if self.mission_source == "fcu":
            return self._fcu_mission_ready()
        return rospy.get_param(self.mission_ready_param, False)

    def _current_mission_progress_index(self):
        if self.mission_source == "fcu":
            return self.fcu_current_target_seq
        return self.active_waypoint_index

    @staticmethod
    def _normalize_mode_names(raw_value):
        if isinstance(raw_value, str):
            values = raw_value.split(",")
        elif isinstance(raw_value, (list, tuple, set)):
            values = raw_value
        else:
            values = []

        return {str(value).strip().upper() for value in values if str(value).strip()}

    def _is_return_mode(self, mode_name):
        if not mode_name:
            return False
        return str(mode_name).strip().upper() in self.return_mode_names

    def _mode_to_resume_after_avoidance(self):
        if self.state_msg is None or not self.state_msg.mode:
            return self.auto_mode

        current_mode = str(self.state_msg.mode).strip()
        if current_mode == self.guided_mode:
            return self.auto_mode

        return current_mode

    def _current_return_target_waypoint_fcu(self):
        if self.fcu_home_position is None:
            rospy.loginfo_throttle(
                1.0,
                "mission_auto_avoid: waiting for FCU home position before handling return-mode obstacle management.",
            )
            return None

        current_pos = self._odom_position()
        # Preserve current altitude during avoidance; FCU resumes its own RTL/RTN profile after GUIDED exits.
        return (self.fcu_home_position[0], self.fcu_home_position[1], current_pos[2])

    def _maybe_sync_fcu_current_before_auto_resume(self):
        if (
            self.mission_source != "fcu"
            or not self.sync_fcu_current_on_auto_resume
            or not self._fcu_mission_ready()
            or self.set_current_client is None
        ):
            return

        current_seq = int(self.fcu_waypoint_list.current_seq)
        current_pos = self._odom_position()
        candidate_seq = None
        candidate_distance = None

        for waypoint in self.fcu_nav_waypoints:
            seq = waypoint["seq"]
            if seq <= current_seq:
                continue

            distance = self._distance(current_pos, waypoint["point"])
            if distance <= self.fcu_set_current_dist_threshold:
                candidate_seq = seq
                candidate_distance = distance
                break

        if candidate_seq is None:
            return

        try:
            rospy.wait_for_service(self.fcu_set_current_service, timeout=1.0)
            response = self.set_current_client(wp_seq=candidate_seq)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn(
                "mission_auto_avoid: failed to sync FCU current mission seq to %d before AUTO resume: %s",
                candidate_seq,
                exc,
            )
            return

        if response.success:
            self.fcu_waypoint_list.current_seq = candidate_seq
            rospy.logwarn(
                "mission_auto_avoid: synced FCU current mission seq %d -> %d before AUTO resume (distance=%.2f m).",
                current_seq,
                candidate_seq,
                candidate_distance,
            )
        else:
            rospy.logwarn(
                "mission_auto_avoid: FCU rejected mission current seq sync %d -> %d before AUTO resume.",
                current_seq,
                candidate_seq,
            )

    def _sync_planning_enabled(self):
        desired_enabled = self._guided_requests_allowed() and (
            self.avoidance_active or self.state_msg.mode == self.guided_mode
        )
        self._set_planning_enabled(desired_enabled, timeout=0.2)

    def _update_rejoin_state(self, route_distance):
        if self.rejoin_active:
            if route_distance <= self.route_rejoin_tolerance:
                self.rejoin_active = False
                rospy.loginfo(
                    "mission_auto_avoid: route rejoin released at %.3fm (<= %.3fm).",
                    route_distance,
                    self.route_rejoin_tolerance,
                )
        else:
            if route_distance >= self.route_rejoin_enter_tolerance:
                self.rejoin_active = True
                rospy.loginfo(
                    "mission_auto_avoid: route rejoin engaged at %.3fm (>= %.3fm).",
                    route_distance,
                    self.route_rejoin_enter_tolerance,
                )

        return self.rejoin_active

    def _set_planning_enabled(self, enabled, timeout=None):
        if self.planning_enabled_state is enabled:
            return True

        wait_timeout = self.planning_service_timeout if timeout is None else timeout

        try:
            rospy.wait_for_service(self.planning_service, timeout=wait_timeout)
            response = self.set_planning_client(enabled)
        except rospy.ROSException as exc:
            rospy.logwarn_throttle(
                2.0,
                "mission_auto_avoid: planner service %s is unavailable: %s" % (self.planning_service, exc),
            )
            return False
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(
                2.0,
                "mission_auto_avoid: set_planning_enabled(%s) failed: %s" % (enabled, exc),
            )
            return False

        if not response.success:
            rospy.logwarn_throttle(
                2.0,
                "mission_auto_avoid: planner service rejected state %s: %s" % (enabled, response.message),
            )
            return False

        self.planning_enabled_state = enabled
        rospy.loginfo("mission_auto_avoid: planner %s.", "enabled" if enabled else "disabled")
        return True

    def _odom_position(self):
        pose = self.odom_msg.pose.pose.position
        return (pose.x, pose.y, pose.z)

    def _odom_speed(self):
        twist = self.odom_msg.twist.twist.linear
        return self._norm((twist.x, twist.y, twist.z))

    @staticmethod
    def _norm(vec):
        return math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])

    @staticmethod
    def _distance(p0, p1):
        return math.sqrt(
            (p0[0] - p1[0]) * (p0[0] - p1[0])
            + (p0[1] - p1[1]) * (p0[1] - p1[1])
            + (p0[2] - p1[2]) * (p0[2] - p1[2])
        )

    @staticmethod
    def _same_point(p0, p1, tolerance=1e-3):
        if p0 is None or p1 is None:
            return False
        return AutoAvoidManager._distance(p0, p1) <= tolerance

    @staticmethod
    def _obstacle_source(route_obstacle_ahead, local_obstacle_ahead):
        if route_obstacle_ahead and local_obstacle_ahead:
            return "route+local"
        if route_obstacle_ahead:
            return "route"
        if local_obstacle_ahead:
            return "local"
        return "none"

    def _on_shutdown(self):
        self.avoidance_active_pub.publish(Bool(False))
        self._set_planning_enabled(False, timeout=1.0)


if __name__ == "__main__":
    rospy.init_node("auto_avoid_manager")
    AutoAvoidManager()
    rospy.spin()
