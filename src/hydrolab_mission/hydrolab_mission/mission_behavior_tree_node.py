import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleOdometry
import py_trees


from hydrolab_mission.behaviors.check_odometry_behavior import CheckOdometry
from hydrolab_mission.behaviors.takeoff_behavior import Takeoff
from hydrolab_mission.behaviors.fly_to_pool_behavior import FlyToPool

from hydrolab_mission.behaviors.check_time_limit_behavior import CheckTimeLimit
from hydrolab_mission.behaviors.return_to_launch_behavior import ReturnToLaunch
from hydrolab_mission.behaviors.has_water_sample_behavior import HasWaterSample
from hydrolab_mission.behaviors.approach_pool_behavior import ApproachPool
from hydrolab_mission.behaviors.descend_over_pool_behavior import DescendOverPool
from hydrolab_mission.behaviors.collect_water_sample_behavior import CollectWaterSample
from hydrolab_mission.behaviors.verify_sample_collected_behavior import VerifySampleCollected
from hydrolab_mission.behaviors.return_to_base_behavior import ReturnToBase
from hydrolab_mission.behaviors.toggle_pool_detection_decorator import TogglePoolDetection

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
        self.declare_parameter('forward_distance_m', 5.0)
        self.declare_parameter('position_tolerance_m', 0.35)
        self.declare_parameter('approach_center_tolerance_u', 0.08)
        self.declare_parameter('approach_center_tolerance_v', 0.08)
        self.declare_parameter('approach_lost_timeout_s', 2.0)
        self.declare_parameter('approach_center_hold_s', 1.0)
        self.declare_parameter('approach_final_speed', 0.6)
        self.declare_parameter('approach_final_length', 3.0)
        self.declare_parameter('initial_approach_center_tolerance_u', 0.16)
        self.declare_parameter('initial_approach_center_tolerance_v', 0.16)
        self.declare_parameter('initial_approach_center_hold_s', 0.8)
        self.declare_parameter('descent_center_tolerance_u', 0.08)
        self.declare_parameter('descent_center_tolerance_v', 0.08)
        self.declare_parameter('descent_step_m', 0.15)
        self.declare_parameter('descent_command_period_s', 0.4)
        self.declare_parameter('descent_target_altitude_m', 0.0)
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
        self.approach_final_length = max(
            float(self.get_parameter('approach_final_length').value),
            0.1,
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
        self.descent_target_altitude_m = float(
            self.get_parameter('descent_target_altitude_m').value
        )
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
        self.blackboard.home_position = None
        self.blackboard.current_position = None
        self.blackboard.gimbal_status = "STATUS_IDLE"
        self.current_gimbal_pitch_deg = None
        self.latest_ball_detection = {'active': False, 'detections': []}
        self.latest_ball_detection_ns = 0

        self.root = self.create_behavior_tree()
        self.tree = py_trees.trees.BehaviourTree(self.root)
        
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

        # Step 1: Action: Takeoff
        takeoff = Takeoff(name=f"Takeoff to {self.takeoff_height_m}m", node=self, target_z=-self.takeoff_height_m)
        main_mission.add_child(takeoff)

        # Pool legs (order and stop color come from mission params)
        for color in self.mission_pool_colors:
            display_color = color.capitalize()
            pool_logic = py_trees.composites.Selector(
                name=f"{display_color} Pool Logic",
                memory=False,
            )

            has_sample = HasWaterSample(
                name=f"Has Water Sample ({display_color})?",
                node=self,
                pool_color=display_color,
            )

            sampling_sequence = py_trees.composites.Sequence(
                name=f"{display_color} Pool Sampling",
                memory=True,
            )
            pool_xy = self.get_pool_xy(color)
            fly_to_pool = FlyToPool(
                name=f"Fly To {display_color} Pool",
                node=self,
                forward_distance=self.forward_distance_m,
                search_z=-self.takeoff_height_m,
                pool_x_m=pool_xy[0] if pool_xy else None,
                pool_y_m=pool_xy[1] if pool_xy else None,
            )
            approach_pool = ApproachPool(
                name=f"Approach {display_color} Pool",
                node=self,
                pool_color=color,
                final_speed_m_s=self.approach_final_speed,
                final_length_m=self.approach_final_length,
            )
            descend_over_pool = DescendOverPool(
                name=f"Descend Over {display_color} Pool",
                node=self,
                pool_color=color,
            )
            approach_sequence = py_trees.composites.Sequence(
                name=f"{display_color} Pool Approach",
                memory=True,
            )
            approach_sequence.add_children([approach_pool, descend_over_pool])
            pool_tracking_sequence = py_trees.composites.Sequence(
                name=f"{display_color} Pool Tracking",
                memory=True,
            )
            pool_tracking_sequence.add_children(
                [fly_to_pool, approach_sequence]
            )
            pool_tracking_with_detection = TogglePoolDetection(
                name=f"Detect {display_color} Pool",
                child=pool_tracking_sequence,
                node=self,
                pool_color=color,
            )
            collect_sample = CollectWaterSample(
                name=f"Collect Sample ({display_color})"
            )
            verify_sample = VerifySampleCollected(
                name=f"Verify Sample ({display_color})"
            )

            sampling_sequence.add_children(
                [
                    pool_tracking_with_detection,
                    collect_sample,
                    verify_sample,
                ]
            )

            retry_decorator = py_trees.decorators.Retry(
                name=f"Retry {display_color} Sampling",
                child=sampling_sequence,
                num_failures=3,
            )

            pool_logic.add_children([has_sample, retry_decorator])
            main_mission.add_child(pool_logic)

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
        msg.data = request_str.lower()
        self.ball_detection_request_pub.publish(msg)

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

    def tick_tree(self):
        self.tree.tick()

def main(args=None):
    rclpy.init(args=args)
    node = MissionBehaviorTree()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
