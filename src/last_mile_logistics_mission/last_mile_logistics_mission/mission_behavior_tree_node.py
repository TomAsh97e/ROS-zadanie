import json

from geometry_msgs.msg import Point, Vector3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleOdometry
import py_trees


from last_mile_logistics_mission.behaviors.check_odometry_behavior import CheckOdometry
from last_mile_logistics_mission.behaviors.takeoff_behavior import Takeoff
from last_mile_logistics_mission.behaviors.search_for_balls_behavior import SearchForBalls

from last_mile_logistics_mission.behaviors.check_time_limit_behavior import CheckTimeLimit
from last_mile_logistics_mission.behaviors.return_to_launch_behavior import ReturnToLaunch
from last_mile_logistics_mission.behaviors.has_ball_behavior import HasBall
from last_mile_logistics_mission.behaviors.approach_ball_behavior import ApproachBall
from last_mile_logistics_mission.behaviors.stage_above_ball_behavior import StageAboveBall
from last_mile_logistics_mission.behaviors.grab_ball_behavior import GrabBall
from last_mile_logistics_mission.behaviors.verify_ball_grabbed_behavior import VerifyBallGrabbed
from last_mile_logistics_mission.behaviors.delivery_behavior import Delivery
from last_mile_logistics_mission.behaviors.toggle_ball_detection_decorator import ToggleBallDetection

class MissionBehaviorTree(Node):
    def __init__(self):
        super().__init__('mission_behavior_tree')
        
        # Declare parameters
        self.declare_parameter('takeoff_height_m', 2.0)
        self.declare_parameter('behavior_tree_tick_s', 0.2)
        self.declare_parameter('forward_distance_m', 5.0)
        self.declare_parameter('blue_search_height_m', 3.0)
        self.declare_parameter('red_search_height_m', 2.0)
        self.declare_parameter('yellow_search_height_m', 2.0)
        self.declare_parameter('blue_search_lateral_offset_m', 0.0)
        self.declare_parameter('red_search_lateral_offset_m', 2.0)
        self.declare_parameter('yellow_search_lateral_offset_m', -2.0)
        self.declare_parameter('downward_scan_hold_s', 1.0)
        self.declare_parameter('early_blue_detection_distance_m', 1.0)
        self.declare_parameter('stage_height_m', 0.5)
        self.declare_parameter('stage_altitude_tolerance_m', 0.12)
        self.declare_parameter('stage_hold_s', 0.6)
        self.declare_parameter('pickup_height_m', 0.35)
        self.declare_parameter('pickup_altitude_tolerance_m', 0.18)
        self.declare_parameter('pickup_center_tolerance_u', 0.035)
        self.declare_parameter('pickup_center_tolerance_v', 0.05)
        self.declare_parameter('pickup_center_hold_s', 0.4)
        self.declare_parameter('position_tolerance_m', 0.35)
        self.declare_parameter('gimbal_down_pitch_deg', -90.0)
        self.declare_parameter('gimbal_yaw_deg', 0.0)
        self.declare_parameter('approach_gimbal_down_tolerance_deg', 6.0)
        self.declare_parameter('approach_center_tolerance_u', 0.12)
        self.declare_parameter('approach_center_tolerance_v', 0.10)
        self.declare_parameter('approach_descend_trigger_v', 0.28)
        self.declare_parameter('approach_lost_timeout_s', 2.0)
        self.declare_parameter('approach_center_hold_s', 0.35)
        self.declare_parameter('ball_grab_verify_timeout_s', 6.5)

        # Retrieve parameters
        self.takeoff_height_m = self.get_parameter('takeoff_height_m').value
        self.behavior_tree_tick_s = max(
            float(self.get_parameter('behavior_tree_tick_s').value),
            0.02,
        )
        self.forward_distance_m = self.get_parameter('forward_distance_m').value
        self.search_heights_m = {
            'blue': max(
                float(self.get_parameter('blue_search_height_m').value),
                0.5,
            ),
            'red': max(
                float(self.get_parameter('red_search_height_m').value),
                0.5,
            ),
            'yellow': max(
                float(self.get_parameter('yellow_search_height_m').value),
                0.5,
            ),
        }
        self.search_lateral_offsets = {
            'blue': float(
                self.get_parameter('blue_search_lateral_offset_m').value
            ),
            'red': float(
                self.get_parameter('red_search_lateral_offset_m').value
            ),
            'yellow': float(
                self.get_parameter('yellow_search_lateral_offset_m').value
            ),
        }
        self.downward_scan_hold_s = float(
            self.get_parameter('downward_scan_hold_s').value
        )
        self.early_blue_detection_distance_m = max(
            float(self.get_parameter('early_blue_detection_distance_m').value),
            0.0,
        )
        self.stage_height_m = float(
            self.get_parameter('stage_height_m').value
        )
        self.stage_altitude_tolerance_m = float(
            self.get_parameter('stage_altitude_tolerance_m').value
        )
        self.stage_hold_s = float(
            self.get_parameter('stage_hold_s').value
        )
        self.pickup_height_m = float(
            self.get_parameter('pickup_height_m').value
        )
        self.pickup_altitude_tolerance_m = float(
            self.get_parameter('pickup_altitude_tolerance_m').value
        )
        self.pickup_center_tolerance_u = float(
            self.get_parameter('pickup_center_tolerance_u').value
        )
        self.pickup_center_tolerance_v = float(
            self.get_parameter('pickup_center_tolerance_v').value
        )
        self.pickup_center_hold_s = float(
            self.get_parameter('pickup_center_hold_s').value
        )
        self.position_tolerance_m = float(
            self.get_parameter('position_tolerance_m').value
        )
        self.approach_gimbal_down_tolerance_deg = float(
            self.get_parameter('approach_gimbal_down_tolerance_deg').value
        )
        self.approach_center_tolerance_u = float(
            self.get_parameter('approach_center_tolerance_u').value
        )
        self.approach_center_tolerance_v = float(
            self.get_parameter('approach_center_tolerance_v').value
        )
        self.approach_descend_trigger_v = float(
            self.get_parameter('approach_descend_trigger_v').value
        )
        self.approach_lost_timeout_s = float(
            self.get_parameter('approach_lost_timeout_s').value
        )
        self.approach_center_hold_s = float(
            self.get_parameter('approach_center_hold_s').value
        )
        self.ball_grab_verify_timeout_s = float(
            self.get_parameter('ball_grab_verify_timeout_s').value
        )
        self.gimbal_down_pitch_deg = float(
            self.get_parameter('gimbal_down_pitch_deg').value
        )
        self.gimbal_yaw_deg = float(
            self.get_parameter('gimbal_yaw_deg').value
        )

        self.target_publisher = self.create_publisher(Point, '/pilot/target_position', 10)
        self.ball_detection_request_pub = self.create_publisher(String, '/vision/ball_detection/request', 10)
        self.ball_mode_pub = self.create_publisher(String, '/pilot/ball_mode', 10)
        self.ball_grabber_enable_pub = self.create_publisher(
            Bool,
            '/ball_grabber/enabled',
            10,
        )
        self.ball_grabber_target_pub = self.create_publisher(
            String,
            '/ball_grabber/target_color',
            10,
        )

        # Gimbal commanding and status tracking
        self.gimbal_command_pub = self.create_publisher(String, '/gimbal/command', 10)
        self.gimbal_target_pub = self.create_publisher(
            Vector3,
            '/gimbal/target_orientation_deg',
            10,
        )
        self.gimbal_status_sub = self.create_subscription(String, '/gimbal/status', self.gimbal_status_callback, 10)
        self.gimbal_attitude_sub = self.create_subscription(
            Vector3,
            '/gimbal/current_orientation_deg',
            self.gimbal_attitude_callback,
            10,
        )
        self.ball_detection_sub = self.create_subscription(
            String,
            '/vision/ball_detection/result',
            self.ball_detection_callback,
            10,
        )
        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        self.ball_grabber_state_sub = self.create_subscription(
            String,
            '/ball_grabber/state',
            self.ball_grabber_state_callback,
            state_qos,
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
        self.current_gimbal_roll_deg = 0.0
        self.current_gimbal_pitch_deg = self.gimbal_down_pitch_deg
        self.current_gimbal_yaw_deg = self.gimbal_yaw_deg
        self.latest_ball_detection = {'active': False, 'detections': []}
        self.latest_ball_detection_ns = 0
        self.ball_grabber_state = {
            'enabled': False,
            'target_color': 'all',
            'collected_colors': [],
            'collected_balls': [],
            'last_collected_color': None,
        }
        self.last_logged_payload_count = 0

        self.root = self.create_behavior_tree()
        self.tree = py_trees.trees.BehaviourTree(self.root)
        
        self.timer = self.create_timer(self.behavior_tree_tick_s, self.tick_tree)
        self.get_logger().info("Behavior Tree Node Initialized.")

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

        # Steps 2, 3, 4: Ball Logic (Selectors)
        for color in ["Blue", "Yellow", "Red"]:
            ball_logic = py_trees.composites.Selector(name=f"{color} Ball Logic", memory=False)
            
            # Condition: Do we already have the Ball?
            has_ball = HasBall(name=f"Has {color} Ball?", node=self, color=color)
            
            # Sequence: Ball Pickup
            pickup_sequence = py_trees.composites.Sequence(name=f"{color} Ball Pickup", memory=True)
            search = SearchForBalls(
                name=f"Search {color}",
                node=self,
                color=color.lower(),
                forward_distance=self.forward_distance_m,
                lateral_offset=self.search_lateral_offsets[color.lower()],
                search_z=-self.search_heights_m[color.lower()],
                position_tolerance_m=self.position_tolerance_m,
                downward_scan_hold_s=self.downward_scan_hold_s,
                early_detection_distance_m=self.early_blue_detection_distance_m,
            )
            approach = ApproachBall(name=f"Approach {color}", node=self, color=color.lower())
            stage = StageAboveBall(
                name=f"Stage Above {color}",
                node=self,
                color=color.lower(),
                stage_height_m=self.stage_height_m,
                altitude_tolerance_m=self.stage_altitude_tolerance_m,
                hold_s=self.stage_hold_s,
            )
            grab = GrabBall(
                name=f"Grab {color}",
                node=self,
                color=color.lower(),
                pickup_height_m=self.pickup_height_m,
                altitude_tolerance_m=self.pickup_altitude_tolerance_m,
            )
            verify = VerifyBallGrabbed(
                name=f"Verify {color}",
                node=self,
                color=color.lower(),
                timeout_s=self.ball_grab_verify_timeout_s,
            )
            
            pickup_sequence.add_children([search, approach, stage, grab, verify])
            
            # Decorate the sequence to toggle ball detection ON when started and OFF when finished
            toggled_sequence = ToggleBallDetection(name=f"Toggle {color} Detection", child=pickup_sequence, node=self, color=color)
            
            # Decorator: Retry (Max 3 attempts) over the toggled sequence
            retry_decorator = py_trees.decorators.Retry(name=f"Retry {color} Pickup", child=toggled_sequence, num_failures=3)
            
            ball_logic.add_children([has_ball, retry_decorator])
            main_mission.add_child(ball_logic)

        # Step 5: Delivery
        delivery = Delivery(name="Delivery Sequence")
        main_mission.add_child(delivery)
        
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
            self.get_logger().info("\n" + "="*55 + "\n>>> 🎯 DRONE LOCKED ON TO THE BALL TARGET! 🎯 <<<\n" + "="*55)
        elif (
            self.blackboard.gimbal_status == "STATUS_LOCKED"
            and new_status not in ("STATUS_LOCKED", "STATUS_IDLE", "STATUS_HOLD")
        ):
            self.get_logger().warn("\n" + "="*55 + "\n>>> ⚠️ DRONE LOST LOCK ON TARGET! ⚠️ <<<\n" + "="*55)
            
        self.blackboard.gimbal_status = new_status

    def gimbal_attitude_callback(self, msg):
        self.current_gimbal_roll_deg = float(msg.x)
        self.current_gimbal_pitch_deg = float(msg.y)
        self.current_gimbal_yaw_deg = float(msg.z)

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

    def ball_grabber_state_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Ignoring invalid ball grabber payload.',
                throttle_duration_sec=2.0,
            )
            return

        self.ball_grabber_state = payload
        payload_count = int(payload.get('payload_count', 0))
        if payload_count > self.last_logged_payload_count:
            collected_balls = payload.get('collected_balls', [])
            order = [
                str(record.get('color', '?')).lower()
                for record in collected_balls
                if record.get('color')
            ]
            if order:
                self.get_logger().info(
                    f'Pickup order so far: {" -> ".join(order)}'
                )
            self.last_logged_payload_count = payload_count

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

    def hold_gimbal_down(self):
        target_msg = Vector3()
        target_msg.x = 0.0
        target_msg.y = float(self.gimbal_down_pitch_deg)
        target_msg.z = float(self.gimbal_yaw_deg)
        self.gimbal_target_pub.publish(target_msg)
        self.send_gimbal_command('COMMAND_HOLD')

    def is_gimbal_pointing_down_for_descent(self):
        return (
            abs(self.current_gimbal_pitch_deg - self.gimbal_down_pitch_deg)
            <= self.approach_gimbal_down_tolerance_deg
        )

    def request_ball_detection(self, request_str):
        msg = String()
        msg.data = request_str.lower()
        self.ball_detection_request_pub.publish(msg)

    def publish_ball_mode(self, mode_str):
        msg = String()
        msg.data = mode_str.lower()
        self.ball_mode_pub.publish(msg)

    def set_ball_grabber_enabled(self, enabled, color=None):
        if color is None and not enabled:
            color = 'all'

        if color is not None:
            color_msg = String()
            color_msg.data = str(color).lower()
            self.ball_grabber_target_pub.publish(color_msg)

        enable_msg = Bool()
        enable_msg.data = bool(enabled)
        self.ball_grabber_enable_pub.publish(enable_msg)

    def has_collected_ball(self, color_name):
        collected_colors = self.ball_grabber_state.get('collected_colors', [])
        normalized_colors = {
            str(color).lower()
            for color in collected_colors
        }
        return color_name.lower() in normalized_colors

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
