import math
from threading import Lock

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class BallGrabberNode(Node):
    def __init__(self):
        super().__init__('ball_grabber')

        self.declare_parameter('world_name', 'hydrolab_built')
        self.declare_parameter('drone_model_name', 'x500_gimbal_0')
        self.declare_parameter(
            'ball_model_names',
            ['ball_red', 'ball_blue', 'ball_yellow'],
        )
        self.declare_parameter('attach_xy_threshold_m', 0.35)
        self.declare_parameter('attach_height_threshold_m', 3.0)
        self.declare_parameter('carry_offset_x_m', 0.0)
        self.declare_parameter('carry_offset_y_m', 0.0)
        self.declare_parameter('carry_offset_z_m', -0.18)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('service_timeout_ms', 100)
        self.declare_parameter('enabled_at_start', False)

        self.world_name = str(self.get_parameter('world_name').value)
        self.drone_model_name = str(
            self.get_parameter('drone_model_name').value
        )
        self.ball_model_names = set(
            self.get_parameter('ball_model_names').value
        )
        self.attach_xy_threshold_m = max(
            float(self.get_parameter('attach_xy_threshold_m').value),
            0.05,
        )
        self.attach_height_threshold_m = max(
            float(self.get_parameter('attach_height_threshold_m').value),
            0.05,
        )
        self.carry_offset_x_m = float(
            self.get_parameter('carry_offset_x_m').value
        )
        self.carry_offset_y_m = float(
            self.get_parameter('carry_offset_y_m').value
        )
        self.carry_offset_z_m = float(
            self.get_parameter('carry_offset_z_m').value
        )
        self.update_rate_hz = max(
            float(self.get_parameter('update_rate_hz').value),
            1.0,
        )
        self.service_timeout_ms = max(
            int(self.get_parameter('service_timeout_ms').value),
            20,
        )
        self.acquire_enabled = bool(
            self.get_parameter('enabled_at_start').value
        )

        self.pose_topic = f'/world/{self.world_name}/pose/info'
        self.set_pose_service = f'/world/{self.world_name}/set_pose'
        self.gz_node = GzNode()

        self.state_lock = Lock()
        self.drone_pose = None
        self.ball_poses = {}
        self.attached_ball = None
        self.last_service_warning_ns = 0

        self.enable_subscription = self.create_subscription(
            Bool,
            '/ball_grabber/enabled',
            self.enable_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.update_rate_hz,
            self.timer_callback,
        )

        success = self.gz_node.subscribe(
            Pose_V,
            self.pose_topic,
            self.pose_callback,
        )
        if not success:
            self.get_logger().error(
                f'Failed to subscribe to Gazebo pose topic: {self.pose_topic}'
            )
        else:
            self.get_logger().info(
                f'Ball Grabber subscribed to pose topic: {self.pose_topic}'
            )

    def enable_callback(self, msg):
        self.acquire_enabled = bool(msg.data)
        state_label = 'enabled' if self.acquire_enabled else 'disabled'
        self.get_logger().info(f'Ball grabber acquisition {state_label}.')

    def pose_callback(self, msg):
        with self.state_lock:
            for pose in msg.pose:
                short_name = self.short_name(pose.name)
                pose_data = {
                    'entity_name': pose.name,
                    'x': float(pose.position.x),
                    'y': float(pose.position.y),
                    'z': float(pose.position.z),
                    'qx': float(pose.orientation.x),
                    'qy': float(pose.orientation.y),
                    'qz': float(pose.orientation.z),
                    'qw': float(pose.orientation.w),
                }

                if short_name == self.drone_model_name:
                    self.drone_pose = pose_data
                elif short_name in self.ball_model_names:
                    self.ball_poses[short_name] = pose_data

    def timer_callback(self):
        if self.attached_ball is None:
            if not self.acquire_enabled:
                return
            self.try_attach_ball()
            return

        self.update_attached_ball_pose()

    def try_attach_ball(self):
        with self.state_lock:
            drone_pose = None if self.drone_pose is None else self.drone_pose.copy()
            ball_poses = {
                key: value.copy()
                for key, value in self.ball_poses.items()
            }

        if drone_pose is None:
            return

        best_ball_name = None
        best_ball_pose = None
        best_distance = None

        for ball_name, ball_pose in ball_poses.items():
            horizontal_distance = math.hypot(
                drone_pose['x'] - ball_pose['x'],
                drone_pose['y'] - ball_pose['y'],
            )
            vertical_distance = drone_pose['z'] - ball_pose['z']

            if horizontal_distance > self.attach_xy_threshold_m:
                continue
            if vertical_distance < 0.0:
                continue
            if vertical_distance > self.attach_height_threshold_m:
                continue

            if best_distance is None or horizontal_distance < best_distance:
                best_distance = horizontal_distance
                best_ball_name = ball_name
                best_ball_pose = ball_pose

        if best_ball_name is None or best_ball_pose is None:
            return

        self.attached_ball = {
            'short_name': best_ball_name,
            'entity_name': best_ball_pose['entity_name'],
        }
        self.get_logger().info(
            f'Attached ball "{best_ball_name}" to drone carrier logic.'
        )

    def update_attached_ball_pose(self):
        with self.state_lock:
            drone_pose = None if self.drone_pose is None else self.drone_pose.copy()
            attached_ball = None
            if self.attached_ball is not None:
                attached_ball = self.attached_ball.copy()

        if drone_pose is None or attached_ball is None:
            return

        target_pose = Pose()
        target_pose.name = attached_ball['entity_name']
        target_pose.position.x = drone_pose['x'] + self.carry_offset_x_m
        target_pose.position.y = drone_pose['y'] + self.carry_offset_y_m
        target_pose.position.z = max(
            drone_pose['z'] + self.carry_offset_z_m,
            0.05,
        )
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0

        success, response = self.gz_node.request(
            self.set_pose_service,
            target_pose,
            Pose,
            Boolean,
            self.service_timeout_ms,
        )
        if success and response.data:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_service_warning_ns >= int(2e9):
            self.get_logger().warn(
                'Failed to update ball pose in Gazebo. '
                f'Service: {self.set_pose_service}'
            )
            self.last_service_warning_ns = now_ns

    def short_name(self, entity_name):
        return entity_name.split('::')[-1]


def main(args=None):
    rclpy.init(args=args)
    node = BallGrabberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
