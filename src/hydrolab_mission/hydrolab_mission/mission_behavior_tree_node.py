import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleOdometry
import py_trees


from hydrolab_mission.behaviors.check_odometry_behavior import CheckOdometry
from hydrolab_mission.behaviors.check_pilot_ready_behavior import CheckPilotReady
from hydrolab_mission.behaviors.takeoff_behavior import Takeoff
from hydrolab_mission.behaviors.fly_to_pool_behavior import FlyToPool

from hydrolab_mission.behaviors.check_time_limit_behavior import CheckTimeLimit
from hydrolab_mission.behaviors.return_to_launch_behavior import ReturnToLaunch
from hydrolab_mission.behaviors.has_water_sample_behavior import HasWaterSample
from hydrolab_mission.behaviors.approach_pool_behavior import ApproachPool
from hydrolab_mission.behaviors.capture_photo_at_stable_hover_behavior import (
    CapturePhotoAtStableHover,
)
from hydrolab_mission.behaviors.descend_over_pool_behavior import DescendOverPool
from hydrolab_mission.behaviors.collect_water_sample_behavior import CollectWaterSample
from hydrolab_mission.behaviors.verify_sample_collected_behavior import VerifySampleCollected
from hydrolab_mission.behaviors.return_to_base_behavior import ReturnToBase
from hydrolab_mission.behaviors.toggle_pool_detection_decorator import TogglePoolDetection
from hydrolab_mission.behaviors.start_search_path_behavior import StartSearchPath
from hydrolab_mission.behaviors.is_target_locked_behavior import IsTargetLocked

POOL_COLORS = ('red', 'green', 'blue', 'yellow')
DEFAULT_POOL_XY = {
    'red': (25.0, 0.0),
    'green': (50.0, 5.0),
    'blue': (75.0, -2.0),
    'yellow': (100.0, 6.0),
}


class MissionBehaviorTree(Node):
    def __init__(self):
        super().__init__('mission_behavior_tree')
        
        # Declare parameters
        self.declare_parameter('takeoff_height_m', 2.0)
        self.declare_parameter('takeoff_start_timeout_s', 4.0)
        self.declare_parameter('takeoff_min_progress_m', 0.2)
        self.declare_parameter('takeoff_retry_attempts', 3)
        self.declare_parameter('forward_distance_m', 5.0)
        self.declare_parameter('position_tolerance_m', 0.35)
        self.declare_parameter('approach_center_tolerance_u', 0.08)
        self.declare_parameter('approach_center_tolerance_v', 0.08)
        self.declare_parameter('approach_lost_timeout_s', 2.0)
        self.declare_parameter('approach_center_hold_s', 1.0)
        self.declare_parameter('approach_final_speed', 0.6)
        self.declare_parameter('approach_slowdown_raycast_distance_m', 0.0)
        self.declare_parameter('approach_final_speed_raycast_distance_m', 0.0)
        self.declare_parameter('initial_approach_center_tolerance_u', 0.16)
        self.declare_parameter('initial_approach_center_tolerance_v', 0.16)
        self.declare_parameter('initial_approach_center_hold_s', 0.8)
        self.declare_parameter('descent_center_tolerance_u', 0.08)
        self.declare_parameter('descent_center_tolerance_v', 0.08)
        self.declare_parameter('descent_step_m', 0.15)
        self.declare_parameter('descent_command_period_s', 0.4)
        self.declare_parameter('capture_photo_hover_altitude_m', 4.0)
        self.declare_parameter('descent_target_altitude_m', 0.0)
        self.declare_parameter('target_detected_timeout_s', 1.0)
        self.declare_parameter(
            'behavior_change_log_file',
            '/home/mateusz/AbsoluteEdge/SImulation_Hydrolab/logs/behavior_changes.log',
        )
        self.declare_parameter(
            'pool_color_order',
            ['red', 'green', 'blue', 'yellow'],
        )
        self.declare_parameter('final_pool_color', 'green')
        for color, (default_x, default_y) in DEFAULT_POOL_XY.items():
            self.declare_parameter(f'pool_{color}_x_m', default_x)
            self.declare_parameter(f'pool_{color}_y_m', default_y)

        # Retrieve parameters
        self.takeoff_height_m = self.get_parameter('takeoff_height_m').value
        self.takeoff_start_timeout_s = max(
            float(self.get_parameter('takeoff_start_timeout_s').value),
            0.5,
        )
        self.takeoff_min_progress_m = max(
            float(self.get_parameter('takeoff_min_progress_m').value),
            0.01,
        )
        self.takeoff_retry_attempts = max(
            int(self.get_parameter('takeoff_retry_attempts').value),
            1,
        )
        self.forward_distance_m = self.get_parameter('forward_distance_m').value
        self.position_tolerance_m = max(
            float(self.get_parameter('position_tolerance_m').value),
            0.05,
        )
        self.approach_center_tolerance_u = float(
            self.get_parameter('approach_center_tolerance_u').value
        )
        self.approach_center_tolerance_v = float(
            self.get_parameter('approach_center_tolerance_v').value
        )
        self.approach_lost_timeout_s = float(
            self.get_parameter('approach_lost_timeout_s').value
        )
        self.approach_center_hold_s = float(
            self.get_parameter('approach_center_hold_s').value
        )
        self.approach_final_speed = max(
            float(self.get_parameter('approach_final_speed').value),
            0.05,
        )
        self.approach_slowdown_raycast_distance_m = max(
            float(self.get_parameter('approach_slowdown_raycast_distance_m').value),
            0.0,
        )
        self.approach_final_speed_raycast_distance_m = max(
            float(self.get_parameter('approach_final_speed_raycast_distance_m').value),
            0.0,
        )
        self.initial_approach_center_tolerance_u = float(
            self.get_parameter('initial_approach_center_tolerance_u').value
        )
        self.initial_approach_center_tolerance_v = float(
            self.get_parameter('initial_approach_center_tolerance_v').value
        )
        self.initial_approach_center_hold_s = float(
            self.get_parameter('initial_approach_center_hold_s').value
        )
        self.descent_center_tolerance_u = float(
            self.get_parameter('descent_center_tolerance_u').value
        )
        self.descent_center_tolerance_v = float(
            self.get_parameter('descent_center_tolerance_v').value
        )
        self.descent_step_m = float(
            self.get_parameter('descent_step_m').value
        )
        self.descent_command_period_s = float(
            self.get_parameter('descent_command_period_s').value
        )
        self.capture_photo_hover_altitude_m = float(
            self.get_parameter('capture_photo_hover_altitude_m').value
        )
        self.descent_target_altitude_m = float(
            self.get_parameter('descent_target_altitude_m').value
        )
        self.target_detected_timeout_s = max(
            float(self.get_parameter('target_detected_timeout_s').value),
            0.1,
        )
        self.behavior_change_log_file = str(
            self.get_parameter('behavior_change_log_file').value
        ).strip()
        self.pool_color_order = [
            str(color).strip().lower()
            for color in self.get_parameter('pool_color_order').value
        ]
        self.final_pool_color = str(
            self.get_parameter('final_pool_color').value
        ).strip().lower()
        self.mission_pool_colors = self._pool_colors_up_to_final()
        self.pool_locations = self._load_pool_locations()

        self.target_publisher = self.create_publisher(Point, '/pilot/target_position', 10)
        self.ball_detection_request_pub = self.create_publisher(String, '/vision/ball_detection/request', 10)
        self.ball_mode_pub = self.create_publisher(String, '/pilot/ball_mode', 10)
        self.approach_contract_pub = self.create_publisher(
            String,
            '/mission/approach_contract',
            10,
        )
        self.hover_contract_pub = self.create_publisher(
            String,
            '/mission/hover_contract',
            10,
        )
        
        # Gimbal commanding and status tracking
        self.gimbal_command_pub = self.create_publisher(String, '/gimbal/command', 10)
        self.gimbal_tracker_enable_pub = self.create_publisher(
            Bool,
            '/gimbal/tracker/enable',
            10,
        )
        self.gimbal_status_sub = self.create_subscription(String, '/gimbal/status', self.gimbal_status_callback, 10)
        self.gimbal_orientation_sub = self.create_subscription(
            Vector3,
            '/gimbal/current_orientation_deg',
            self.gimbal_orientation_callback,
            10,
        )
        self.ball_detection_sub = self.create_subscription(
            String,
            '/vision/ball_detection/result',
            self.ball_detection_callback,
            10,
        )
        self.pilot_ready_sub = self.create_subscription(
            Bool,
            '/pilot/ready',
            self.pilot_ready_callback,
            10,
        )

        self.odom_subscriber = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.odometry_callback, 
            qos_profile_sensor_data
        )

        self.blackboard = py_trees.blackboard.Client(name="Master")
        self.blackboard.register_key(key="home_position", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gimbal_status", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="pilot_ready", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_detected", access=py_trees.common.Access.WRITE)
        self.blackboard.home_position = None
        self.blackboard.current_position = None
        self.blackboard.gimbal_status = "STATUS_IDLE"
        self.blackboard.pilot_ready = False
        self.blackboard.target_detected = False
        self.current_gimbal_pitch_deg = None
        self.latest_ball_detection = {'active': False, 'detections': []}
        self.latest_ball_detection_ns = 0
        self.detection_target_color = None
        self.behavior_change_log_path = None
        self._behavior_status_snapshot = {}

        self.root = self.create_behavior_tree()
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self._initialize_behavior_change_logger()
        
        self.timer = self.create_timer(0.2, self.tick_tree)
        self.get_logger().info(
            'Behavior Tree Node Initialized. '
            f'Pool mission colors: {" -> ".join(self.mission_pool_colors)} '
            f'(final: {self.final_pool_color}).'
        )

    def _pool_colors_up_to_final(self):
        if not self.pool_color_order:
            self.get_logger().warn('pool_color_order is empty; defaulting to ["yellow"].')
            return ['yellow']

        if self.final_pool_color not in self.pool_color_order:
            self.get_logger().warn(
                f"final_pool_color '{self.final_pool_color}' not in pool_color_order "
                f'{self.pool_color_order}; running full color order.'
            )
            return list(self.pool_color_order)

        final_index = self.pool_color_order.index(self.final_pool_color)
        return self.pool_color_order[: final_index + 1]

    def _load_pool_locations(self):
        return {
            color: (
                float(self.get_parameter(f'pool_{color}_x_m').value),
                float(self.get_parameter(f'pool_{color}_y_m').value),
            )
            for color in POOL_COLORS
        }

    def get_pool_xy(self, pool_color):
        return self.pool_locations.get(str(pool_color).strip().lower())

    def _make_locked_condition(self, name):
        return IsTargetLocked(
            name=name,
            node=self,
        )

    def _make_pool_acquire_subtree(self, color, post_approach_children=None):
        display_color = color.capitalize()
        pool_xy = self.get_pool_xy(color)
        post_approach_children = list(post_approach_children or [])

        guide_mode = py_trees.composites.Sequence(
            name=f"{display_color} Guide Mode",
            memory=True,
        )
        guide_children = [
            self._make_locked_condition(
                name=f"{display_color} Target Acquired?"
            ),
            ApproachPool(
                name=f"Approach {display_color} Pool",
                node=self,
                pool_color=color,
                final_speed_m_s=self.approach_final_speed,
                slowdown_raycast_distance_m=self.approach_slowdown_raycast_distance_m,
                final_speed_raycast_distance_m=self.approach_final_speed_raycast_distance_m,
            ),
        ]

        if post_approach_children:
            post_approach_actions = py_trees.composites.Sequence(
                name=f"{display_color} Post-Approach Actions",
                memory=True,
            )
            post_approach_actions.add_children(post_approach_children)

            post_approach_guard = py_trees.composites.Sequence(
                name=f"{display_color} Post-Approach Guard",
                memory=False,
            )
            post_approach_guard.add_children(
                [
                    self._make_locked_condition(
                        name=f"{display_color} Target Still Detected?"
                    ),
                    post_approach_actions,
                ]
            )
            guide_children.append(post_approach_guard)

        guide_mode.add_children(guide_children)

        # Search mode: fly to waypoint with gimbal scanning active, then run
        # search path. FlyToPool already returns SUCCESS when gimbal locks, so
        # the reactive Selector above will switch to GuideMode on the very next
        # tick without needing a Parallel.
        search_mode = py_trees.composites.Sequence(
            name=f"{display_color} Search Mode",
            memory=True,
        )
        search_mode.add_children(
            [
                FlyToPool(
                    name=f"Fly To {display_color} Pool",
                    node=self,
                    forward_distance=self.forward_distance_m,
                    search_z=-self.takeoff_height_m,
                    pool_x_m=pool_xy[0] if pool_xy else None,
                    pool_y_m=pool_xy[1] if pool_xy else None,
                ),
                StartSearchPath(
                    name=f"Start {display_color} Search Path"
                ),
            ]
        )

        pool_acquire = py_trees.composites.Selector(
            name=f"{display_color} Pool Acquire",
            memory=False,
        )
        pool_acquire.add_children([guide_mode, search_mode])
        return TogglePoolDetection(
            name=f"Detect {display_color} Pool",
            child=pool_acquire,
            node=self,
            pool_color=color,
        )

    def _make_normal_pool_task(self, color):
        display_color = color.capitalize()
        normal_pool_task = py_trees.composites.Sequence(
            name=f"{display_color} Pool Recon Task",
            memory=True,
        )
        normal_pool_task.add_children(
            [
                self._make_pool_acquire_subtree(
                    color,
                    post_approach_children=[
                        CapturePhotoAtStableHover(
                            name=f"Capture Photo At Stable Hover ({display_color})",
                            node=self,
                            pool_color=color,
                        ),
                    ],
                ),
            ]
        )
        return py_trees.decorators.Retry(
            name=f"Retry {display_color} Recon",
            child=normal_pool_task,
            num_failures=3,
        )

    def _make_final_pool_task(self, color):
        display_color = color.capitalize()
        final_pool_task = py_trees.composites.Sequence(
            name=f"{display_color} Final Pool Task",
            memory=True,
        )
        final_pool_task.add_children(
            [
                self._make_pool_acquire_subtree(
                    color,
                    post_approach_children=[
                        CapturePhotoAtStableHover(
                            name=f"Capture Photo At Stable Hover ({display_color})",
                            node=self,
                            pool_color=color,
                        ),
                        DescendOverPool(
                            name=f"Descend Over {display_color} Pool",
                            node=self,
                            pool_color=color,
                        ),
                        CollectWaterSample(
                            name=f"Collect Sample ({display_color})"
                        ),
                        VerifySampleCollected(
                            name=f"Verify Sample ({display_color})"
                        ),
                    ],
                ),
            ]
        )
        return py_trees.decorators.Retry(
            name=f"Retry {display_color} Sampling",
            child=final_pool_task,
            num_failures=3,
        )

    def create_behavior_tree(self):
        root = py_trees.composites.Selector(name="Root", memory=False)

        # [->] Branch 1: Safety (Sequence)
        safety_sequence = py_trees.composites.Sequence(name="Safety", memory=False)
        check_time = CheckTimeLimit(name="Check Mission Time > 9.5m")
        # Invert the check_time so sequence continues ONLY if Time Limit check FAILS (i.e. > 9.5s)
        time_limit_exceeded = py_trees.decorators.Inverter(name="Is Time Exceeded?", child=check_time)
        rtl = ReturnToLaunch(name="Return to Launch")
        safety_sequence.add_children([time_limit_exceeded, rtl])

        # [->] Branch 2: Main Mission (Sequence)
        main_mission = py_trees.composites.Sequence(name="Main Mission", memory=True)

        check_odom = CheckOdometry(name="Check Odometry", node=self)
        main_mission.add_child(check_odom)

        check_pilot_ready = CheckPilotReady(name="Check Pilot Ready", node=self)
        main_mission.add_child(check_pilot_ready)

        # Step 1: Action: Takeoff
        takeoff = Takeoff(
            name=f"Takeoff to {self.takeoff_height_m}m",
            node=self,
            target_z=-self.takeoff_height_m,
            start_timeout_s=self.takeoff_start_timeout_s,
            min_progress_m=self.takeoff_min_progress_m,
        )
        takeoff_with_retry = py_trees.decorators.Retry(
            name="Retry Takeoff",
            child=takeoff,
            num_failures=self.takeoff_retry_attempts,
        )
        main_mission.add_child(takeoff_with_retry)

        # Pool legs split into recon pools and one final sampling pool.
        normal_pool_colors = self.mission_pool_colors[:-1]
        final_pool_color = self.mission_pool_colors[-1]

        for color in normal_pool_colors:
            main_mission.add_child(self._make_normal_pool_task(color))

        final_display_color = final_pool_color.capitalize()
        final_pool_logic = py_trees.composites.Selector(
            name=f"{final_display_color} Final Pool Logic",
            memory=False,
        )
        final_has_sample = HasWaterSample(
            name=f"Has Water Sample ({final_display_color})?",
            node=self,
            pool_color=final_display_color,
        )
        final_pool_logic.add_children(
            [
                final_has_sample,
                self._make_final_pool_task(final_pool_color),
            ]
        )
        main_mission.add_child(final_pool_logic)

        return_to_base = ReturnToBase(name="Return To Base")
        main_mission.add_child(return_to_base)
        
        root.add_children([safety_sequence, main_mission])
        return root

    def odometry_callback(self, msg):
        current_pos = [msg.position[0], msg.position[1], msg.position[2]]
        self.blackboard.current_position = current_pos

        if self.blackboard.home_position is None:
            self.blackboard.home_position = current_pos

    def gimbal_status_callback(self, msg):
        new_status = msg.data.upper()
        
        # Notify the console prominently when transitioning to or from LOCKED state
        if self.blackboard.gimbal_status != "STATUS_LOCKED" and new_status == "STATUS_LOCKED":
            self.get_logger().info("\n" + "="*55 + "\n>>> 🎯 DRONE LOCKED ON TO THE POOL TARGET! 🎯 <<<\n" + "="*55)
        elif self.blackboard.gimbal_status == "STATUS_LOCKED" and new_status != "STATUS_LOCKED" and new_status != "STATUS_IDLE":
            self.get_logger().warn("\n" + "="*55 + "\n>>> ⚠️ DRONE LOST LOCK ON TARGET! ⚠️ <<<\n" + "="*55)
            
        self.blackboard.gimbal_status = new_status

    def gimbal_orientation_callback(self, msg):
        self.current_gimbal_pitch_deg = float(msg.y)

    def ball_detection_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Ignoring invalid ball detection payload.',
                throttle_duration_sec=2.0,
            )
            return

        self.latest_ball_detection = payload
        self.latest_ball_detection_ns = self.get_clock().now().nanoseconds
        self._refresh_target_detected_status()

    def pilot_ready_callback(self, msg):
        self.blackboard.pilot_ready = bool(msg.data)

    def publish_target(self, x, y, z):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.target_publisher.publish(msg)

    def send_gimbal_command(self, cmd_str):
        msg = String()
        msg.data = cmd_str.upper()
        self.gimbal_command_pub.publish(msg)

    def set_gimbal_tracker_enabled(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.gimbal_tracker_enable_pub.publish(msg)

    def request_ball_detection(self, request_str):
        msg = String()
        request_value = str(request_str).strip().lower()
        msg.data = request_value
        self.ball_detection_request_pub.publish(msg)
        if request_value in ('', 'stop'):
            self.detection_target_color = None
        else:
            self.detection_target_color = request_value
        self._refresh_target_detected_status()

    def publish_ball_mode(self, mode_str):
        msg = String()
        msg.data = mode_str.lower()
        self.ball_mode_pub.publish(msg)

    def publish_approach_contract(self, contract):
        if not isinstance(contract, dict):
            self.get_logger().warn('Ignoring invalid approach contract payload.')
            return

        msg = String()
        msg.data = json.dumps(contract)
        self.approach_contract_pub.publish(msg)

    def publish_hover_contract(self, contract):
        if not isinstance(contract, dict):
            self.get_logger().warn('Ignoring invalid hover contract payload.')
            return

        msg = String()
        msg.data = json.dumps(contract)
        self.hover_contract_pub.publish(msg)

    def get_detection(self, color_name):
        if self.latest_ball_detection_ns == 0:
            return None

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.latest_ball_detection_ns > int(
            self.approach_lost_timeout_s * 1e9
        ):
            return None

        if not self.latest_ball_detection.get('active', False):
            return None

        detections = self.latest_ball_detection.get('detections', [])
        for detection in detections:
            if detection.get('color', '').lower() == color_name.lower():
                return detection

        return None

    def _has_recent_detection_for_active_target(self):
        if self.detection_target_color is None:
            return False

        if self.latest_ball_detection_ns == 0:
            return False

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.latest_ball_detection_ns > int(
            self.target_detected_timeout_s * 1e9
        ):
            return False

        if not self.latest_ball_detection.get('active', False):
            return False

        detections = self.latest_ball_detection.get('detections', [])
        if self.detection_target_color == 'all':
            return bool(detections)

        target_color = self.detection_target_color.lower()
        return any(
            str(detection.get('color', '')).lower() == target_color
            for detection in detections
        )

    def _refresh_target_detected_status(self):
        target_detected = self._has_recent_detection_for_active_target()
        previous_status = bool(getattr(self.blackboard, 'target_detected', False))
        self.blackboard.target_detected = target_detected
        if target_detected != previous_status:
            self.get_logger().info(
                'Target detection status changed: '
                f'target_detected={target_detected}, '
                f'active_target={self.detection_target_color}.'
            )

    def _initialize_behavior_change_logger(self):
        if not self.behavior_change_log_file:
            self.get_logger().warn(
                'behavior_change_log_file is empty; behavior transition logging disabled.'
            )
            return

        try:
            log_path = Path(self.behavior_change_log_file).expanduser()
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open('a', encoding='utf-8') as log_file:
                log_file.write('\n=== mission_behavior_tree session started ===\n')
            self.behavior_change_log_path = log_path
            self.get_logger().info(
                f'Behavior transition log file: {self.behavior_change_log_path}'
            )
        except OSError as exc:
            self.behavior_change_log_path = None
            self.get_logger().error(
                f'Failed to initialize behavior transition log file: {exc}'
            )
            return

        self._behavior_status_snapshot = {
            id(behavior): behavior.status
            for behavior in self._walk_behaviors()
        }

    def _status_name(self, status):
        if status is None:
            return 'NONE'
        return str(getattr(status, 'name', status))

    def _walk_behaviors(self):
        if self.root is None:
            return []

        visited = set()
        ordered = []
        stack = [self.root]
        while stack:
            behavior = stack.pop()
            behavior_id = id(behavior)
            if behavior_id in visited:
                continue
            visited.add(behavior_id)
            ordered.append(behavior)

            children = []
            maybe_children = getattr(behavior, 'children', None)
            if maybe_children:
                children.extend(reversed(maybe_children))
            else:
                decorated = getattr(behavior, 'decorated', None)
                if decorated is not None:
                    children.append(decorated)
                child = getattr(behavior, 'child', None)
                if child is not None and child is not decorated:
                    children.append(child)

            stack.extend(children)

        return ordered

    def _log_behavior_status_changes(self):
        if self.behavior_change_log_path is None:
            return

        timestamp_ns = self.get_clock().now().nanoseconds
        changed_lines = []

        for behavior in self._walk_behaviors():
            behavior_id = id(behavior)
            previous_status = self._behavior_status_snapshot.get(behavior_id)
            current_status = behavior.status
            if previous_status != current_status:
                changed_lines.append(
                    f'{timestamp_ns} | {behavior.name} '
                    f'[{behavior.__class__.__name__}] '
                    f'{self._status_name(previous_status)} -> '
                    f'{self._status_name(current_status)}\n'
                )
            self._behavior_status_snapshot[behavior_id] = current_status

        if not changed_lines:
            return

        try:
            with self.behavior_change_log_path.open('a', encoding='utf-8') as log_file:
                log_file.writelines(changed_lines)
        except OSError as exc:
            self.get_logger().error(
                f'Failed to write behavior transition log file: {exc}'
            )

    def tick_tree(self):
        self._refresh_target_detected_status()
        self.tree.tick()
        self._log_behavior_status_changes()

def main(args=None):
    rclpy.init(args=args)
    node = MissionBehaviorTree()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
