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
from hydrolab_mission.behaviors.search_for_balls_behavior import SearchForBalls

from hydrolab_mission.behaviors.check_time_limit_behavior import CheckTimeLimit
from hydrolab_mission.behaviors.return_to_launch_behavior import ReturnToLaunch
from hydrolab_mission.behaviors.has_ball_behavior import HasBall
from hydrolab_mission.behaviors.initial_approach_ball_behavior import InitialApproachBall
from hydrolab_mission.behaviors.descend_over_ball_behavior import DescendOverBall
from hydrolab_mission.behaviors.grab_ball_behavior import GrabBall
from hydrolab_mission.behaviors.verify_ball_grabbed_behavior import VerifyBallGrabbed
from hydrolab_mission.behaviors.delivery_behavior import Delivery
from hydrolab_mission.behaviors.toggle_ball_detection_decorator import ToggleBallDetection

class MissionBehaviorTree(Node):
    def __init__(self):
        super().__init__('mission_behavior_tree')
        
        # Declare parameters
        self.declare_parameter('takeoff_height_m', 2.0)
        self.declare_parameter('forward_distance_m', 5.0)
        self.declare_parameter('approach_center_tolerance_u', 0.08)
        self.declare_parameter('approach_center_tolerance_v', 0.08)
        self.declare_parameter('approach_lost_timeout_s', 2.0)
        self.declare_parameter('approach_center_hold_s', 1.0)
        self.declare_parameter('initial_approach_center_tolerance_u', 0.16)
        self.declare_parameter('initial_approach_center_tolerance_v', 0.16)
        self.declare_parameter('initial_approach_center_hold_s', 0.8)
        self.declare_parameter('descent_center_tolerance_u', 0.08)
        self.declare_parameter('descent_center_tolerance_v', 0.08)
        self.declare_parameter('descent_step_m', 0.15)
        self.declare_parameter('descent_command_period_s', 0.4)
        self.declare_parameter('descent_target_altitude_m', 0.0)

        # Retrieve parameters
        self.takeoff_height_m = self.get_parameter('takeoff_height_m').value
        self.forward_distance_m = self.get_parameter('forward_distance_m').value
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
        for color in [ "Red", "Blue","Yellow"]:
            ball_logic = py_trees.composites.Selector(name=f"{color} Ball Logic", memory=False)
            
            # Condition: Do we already have the Ball?
            has_ball = HasBall(name=f"Has {color} Ball?", node=self, color=color)
            
            # Sequence: Ball Pickup
            pickup_sequence = py_trees.composites.Sequence(name=f"{color} Ball Pickup", memory=True)
            search = SearchForBalls(name=f"Search {color}", node=self, forward_distance=self.forward_distance_m, search_z=-self.takeoff_height_m)
            approach_sequence = py_trees.composites.Sequence(
                name=f"{color} Approach Sequence",
                memory=True,
            )
            initial_approach = InitialApproachBall(
                name=f"Initial {color} Approach",
                node=self,
                color=color.lower(),
            )
            descent = DescendOverBall(
                name=f"Descent {color}",
                node=self,
                color=color.lower(),
            )
            approach_sequence.add_children([initial_approach, descent])
            grab = GrabBall(name=f"Grab {color}")
            verify = VerifyBallGrabbed(name=f"Verify {color}")
            
            pickup_sequence.add_children([search, approach_sequence, grab, verify])
            
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
