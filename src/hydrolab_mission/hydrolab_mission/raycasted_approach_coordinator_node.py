import json
import math

from geometry_msgs.msg import Point, Vector3
from px4_msgs.msg import DistanceSensor, VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class RaycastedApproachCoordinatorNode(Node):
    def __init__(self):
        super().__init__('raycasted_approach_coordinator_node')

        self.declare_parameter('control_interval_s', 0.1)
        self.declare_parameter('target_topic', '/pilot/target_position')
        self.declare_parameter('approach_contract_topic', '/mission/approach_contract')
        self.declare_parameter('gimbal_status_topic', '/gimbal/status')
        self.declare_parameter('gimbal_orientation_topic', '/gimbal/current_orientation_deg')
        self.declare_parameter('odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('distance_sensor_topic', '/fmu/out/distance_sensor')
        self.declare_parameter('detection_topic', '/vision/ball_detection/result')
        self.declare_parameter('require_gimbal_lock', True)
        self.declare_parameter('min_altitude_m', 0.25)
        self.declare_parameter('min_pitch_down_deg', 5.0)
        self.declare_parameter('camera_hfov_deg', 114.6)
        self.declare_parameter('camera_vfov_deg', 71.6)
        self.declare_parameter('detection_timeout_s', 0.6)
        self.declare_parameter('camera_mount_pitch_offset_deg', 0.0)
        self.declare_parameter('camera_mount_yaw_offset_deg', 0.0)
        self.declare_parameter('camera_offset_x_m', 0.0)
        self.declare_parameter('camera_offset_y_m', 0.0)
        self.declare_parameter('camera_offset_z_m', 0.0)
        self.declare_parameter('max_raycast_distance_m', 25.0)

        self.control_interval_s = max(
            float(self.get_parameter('control_interval_s').value),
            0.02,
        )
        self.require_gimbal_lock = bool(self.get_parameter('require_gimbal_lock').value)
        self.min_altitude_m = max(float(self.get_parameter('min_altitude_m').value), 0.01)
        self.min_pitch_down_deg = max(
            float(self.get_parameter('min_pitch_down_deg').value),
            0.1,
        )
        self.camera_hfov_deg = max(
            float(self.get_parameter('camera_hfov_deg').value),
            1.0,
        )
        self.camera_vfov_deg = max(
            float(self.get_parameter('camera_vfov_deg').value),
            1.0,
        )
        self.detection_timeout_s = max(
            float(self.get_parameter('detection_timeout_s').value),
            0.05,
        )
        self.camera_mount_pitch_offset_deg = float(
            self.get_parameter('camera_mount_pitch_offset_deg').value
        )
        self.camera_mount_yaw_offset_deg = float(
            self.get_parameter('camera_mount_yaw_offset_deg').value
        )
        self.camera_offset_body = [
            float(self.get_parameter('camera_offset_x_m').value),
            float(self.get_parameter('camera_offset_y_m').value),
            float(self.get_parameter('camera_offset_z_m').value),
        ]
        self.max_raycast_distance_m = max(
            float(self.get_parameter('max_raycast_distance_m').value),
            0.1,
        )

        target_topic = str(self.get_parameter('target_topic').value)
        approach_contract_topic = str(
            self.get_parameter('approach_contract_topic').value
        )
        gimbal_status_topic = str(self.get_parameter('gimbal_status_topic').value)
        gimbal_orientation_topic = str(self.get_parameter('gimbal_orientation_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        distance_sensor_topic = str(self.get_parameter('distance_sensor_topic').value)
        detection_topic = str(self.get_parameter('detection_topic').value)

        self.target_publisher = self.create_publisher(Point, target_topic, 10)
        self.create_subscription(
            VehicleOdometry,
            odom_topic,
            self.odometry_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DistanceSensor,
            distance_sensor_topic,
            self.distance_sensor_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3,
            gimbal_orientation_topic,
            self.gimbal_orientation_callback,
            10,
        )
        self.create_subscription(
            String,
            gimbal_status_topic,
            self.gimbal_status_callback,
            10,
        )
        self.create_subscription(
            String,
            approach_contract_topic,
            self.approach_contract_callback,
            10,
        )
        self.create_subscription(
            String,
            detection_topic,
            self.detection_callback,
            10,
        )

        self.home_position = None
        self.current_position = None
        self.body_to_world = None
        self.downward_distance_m = None
        self.downward_sensor_axis_body = [0.0, 0.0, 1.0]

        self.gimbal_pitch_deg = 0.0
        self.gimbal_yaw_deg = 0.0
        self.gimbal_status = 'STATUS_IDLE'
        self.contract_active = False
        self.contract_tracking_mode = 'idle'
        self.contract_target_kind = ''
        self.contract_target_label = ''
        self.contract_target_variant = ''
        self.contract_target_source = ''
        self.contract_require_lock = self.require_gimbal_lock
        self.contract_target_altitude_m = None
        self.target_center_uv = None
        self.last_detection_time_s = 0.0

        self.last_log_ns = 0

        self.timer = self.create_timer(
            self.control_interval_s,
            self.timer_callback,
        )
        self.get_logger().info(
            'Raycasted approach coordinator initialized. '
            f'Publishing to {target_topic}'
        )

    def odometry_callback(self, msg):
        self.current_position = [
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        ]
        if self.home_position is None:
            self.home_position = self.current_position.copy()

        self.body_to_world = self.quaternion_to_matrix(msg.q)

    def distance_sensor_callback(self, msg):
        distance_m = float(msg.current_distance)
        if not math.isfinite(distance_m) or distance_m <= 0.0:
            return
        self.downward_distance_m = distance_m

    def gimbal_orientation_callback(self, msg):
        self.gimbal_pitch_deg = float(msg.y)
        self.gimbal_yaw_deg = float(msg.z)

    def gimbal_status_callback(self, msg):
        self.gimbal_status = msg.data.strip().upper()

    def approach_contract_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Ignoring invalid approach contract payload.',
                throttle_duration_sec=2.0,
            )
            return

        if not isinstance(payload, dict):
            self.get_logger().warn(
                'Ignoring non-object approach contract payload.',
                throttle_duration_sec=2.0,
            )
            return

        self.contract_active = bool(payload.get('active', False))
        self.contract_tracking_mode = str(
            payload.get('tracking_mode', 'idle')
        ).strip().lower()

        target = payload.get('target', {})
        if not isinstance(target, dict):
            target = {}
        self.contract_target_kind = str(target.get('kind', '')).strip().lower()
        self.contract_target_label = str(target.get('label', '')).strip().lower()
        self.contract_target_variant = str(target.get('variant', '')).strip().lower()
        self.contract_target_source = str(target.get('source', '')).strip().lower()

        constraints = payload.get('constraints', {})
        if not isinstance(constraints, dict):
            constraints = {}
        self.contract_require_lock = bool(
            constraints.get('require_lock', self.require_gimbal_lock)
        )
        target_altitude_value = constraints.get('target_altitude_m', None)
        if target_altitude_value is None:
            self.contract_target_altitude_m = None
        else:
            try:
                self.contract_target_altitude_m = max(0.0, float(target_altitude_value))
            except (TypeError, ValueError):
                self.contract_target_altitude_m = None

    def detection_callback(self, msg):
        center_uv = self.extract_target_center_uv(
            msg.data,
            target_label=self.contract_target_label,
            target_variant=self.contract_target_variant,
        )
        if center_uv is None:
            self.target_center_uv = None
            return

        self.target_center_uv = center_uv
        self.last_detection_time_s = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        if (
            self.current_position is None
            or self.home_position is None
            or self.body_to_world is None
        ):
            return

        if not self.contract_active:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.target_center_uv is None:
            return
        if (now_s - self.last_detection_time_s) > self.detection_timeout_s:
            return

        pitch_deg = self.gimbal_pitch_deg + self.camera_mount_pitch_offset_deg

        ray_camera = self.uv_to_camera_ray(
            self.target_center_uv[0],
            self.target_center_uv[1],
        )

        yaw_deg = self.gimbal_yaw_deg + self.camera_mount_yaw_offset_deg
        ray_body = self.rotate_gimbal_to_body(ray_camera, pitch_deg, yaw_deg)
        ray_world = self.mul_mat3_vec3(self.body_to_world, ray_body)

        ray_world_z = ray_world[2]
        if ray_world_z <= 1e-4:
            return

        camera_origin_world = self.add_vec3(
            self.current_position,
            self.mul_mat3_vec3(self.body_to_world, self.camera_offset_body),
        )

        if self.downward_distance_m is None:
            return

        # Tilt compensation:
        # project measured slant range from body-down axis onto world-down.
        downward_axis_world = self.mul_mat3_vec3(
            self.body_to_world,
            self.downward_sensor_axis_body,
        )
        downward_axis_world_z = downward_axis_world[2]
        if downward_axis_world_z <= 1e-4:
            return

        downward_altitude_m = self.downward_distance_m * downward_axis_world_z
        if downward_altitude_m <= 1e-4:
            return

        ground_z = self.current_position[2] + downward_altitude_m
        distance_to_plane_z = ground_z - camera_origin_world[2]
        if distance_to_plane_z <= 1e-4:
            return

        t = distance_to_plane_z / ray_world_z
        if t <= 0.0:
            return

        t = min(t, self.max_raycast_distance_m)
        target_x = camera_origin_world[0] + (t * ray_world[0])
        target_y = camera_origin_world[1] + (t * ray_world[1])
        target_z = self.current_position[2]
        if (
            self.contract_tracking_mode == 'descent'
            and self.contract_target_altitude_m is not None
        ):
            desired_z = self.home_position[2] - self.contract_target_altitude_m
            target_z = max(self.current_position[2], desired_z)

        msg = Point()
        msg.x = float(target_x)
        msg.y = float(target_y)
        msg.z = float(target_z)
        self.target_publisher.publish(msg)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= int(1e9):
            self.get_logger().info(
                'Raycast target: '
                f'uv=({self.target_center_uv[0]:.3f},{self.target_center_uv[1]:.3f}), '
                f'pitch={pitch_deg:.1f} deg, '
                f'yaw={yaw_deg:.1f} deg, '
                f'downward_distance={self.downward_distance_m:.2f} m, '
                f'projected_altitude={downward_altitude_m:.2f} m, '
                f'd={t:.2f} m, '
                f'target=({target_x:.2f}, {target_y:.2f}, {target_z:.2f})'
            )
            self.last_log_ns = now_ns

    def extract_target_center_uv(self, payload_raw, target_label='', target_variant=''):
        try:
            payload = json.loads(payload_raw)
        except json.JSONDecodeError:
            return None

        if not isinstance(payload, dict):
            return None
        if not bool(payload.get('active', False)):
            return None

        detections = payload.get('detections', [])
        if not isinstance(detections, list) or not detections:
            return None

        requested_variant = payload.get('request', '')
        if not isinstance(requested_variant, str):
            requested_variant = ''
        requested_variant = requested_variant.lower()

        label = (target_label or 'ball').lower()
        variant = target_variant.lower()

        target = None
        for detection in detections:
            if not isinstance(detection, dict):
                continue

            detection_label = str(
                detection.get('label', detection.get('kind', 'ball'))
            ).lower()
            if detection_label and detection_label != label:
                continue

            detection_variant = str(
                detection.get('variant', detection.get('color', ''))
            ).lower()
            if variant and detection_variant != variant:
                continue

            if requested_variant == 'all' or not requested_variant:
                target = detection
                break
            if detection_variant == requested_variant:
                target = detection
                break

        if target is None:
            for detection in detections:
                if not isinstance(detection, dict):
                    continue
                detection_label = str(
                    detection.get('label', detection.get('kind', 'ball'))
                ).lower()
                if detection_label == label:
                    target = detection
                    break
            if target is None:
                target = detections[0]
                if not isinstance(target, dict):
                    return None

        center_uv = target.get('center_uv')
        if not isinstance(center_uv, list) or len(center_uv) != 2:
            return None

        try:
            u = float(center_uv[0])
            v = float(center_uv[1])
        except (TypeError, ValueError):
            return None

        if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
            return None
        return [u, v]

    def uv_to_camera_ray(self, u, v):
        half_hfov_rad = 0.5 * math.radians(self.camera_hfov_deg)
        half_vfov_rad = 0.5 * math.radians(self.camera_vfov_deg)

        angle_h_rad = (2.0 * u - 1.0) * half_hfov_rad
        angle_v_rad = (2.0 * v - 1.0) * half_vfov_rad

        # Camera frame convention:
        # x forward, y right, z down.
        ray = [
            1.0,
            math.tan(angle_h_rad),
            math.tan(angle_v_rad),
        ]
        return self.normalize_vec3(ray)

    def rotate_gimbal_to_body(self, ray_camera, pitch_deg, yaw_deg):
        yaw_rad = math.radians(yaw_deg)
        # Gimbal pitch topic convention is negative when looking down.
        pitch_down_rad = math.radians(-pitch_deg)

        ray_after_pitch = [
            (math.cos(pitch_down_rad) * ray_camera[0])
            - (math.sin(pitch_down_rad) * ray_camera[2]),
            ray_camera[1],
            (math.sin(pitch_down_rad) * ray_camera[0])
            + (math.cos(pitch_down_rad) * ray_camera[2]),
        ]
        ray_after_yaw = [
            (math.cos(yaw_rad) * ray_after_pitch[0])
            - (math.sin(yaw_rad) * ray_after_pitch[1]),
            (math.sin(yaw_rad) * ray_after_pitch[0])
            + (math.cos(yaw_rad) * ray_after_pitch[1]),
            ray_after_pitch[2],
        ]
        return self.normalize_vec3(ray_after_yaw)

    @staticmethod
    def quaternion_to_matrix(q):
        qw = float(q[0])
        qx = float(q[1])
        qy = float(q[2])
        qz = float(q[3])
        return [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ]

    @staticmethod
    def mul_mat3_vec3(mat, vec):
        return [
            (mat[0][0] * vec[0]) + (mat[0][1] * vec[1]) + (mat[0][2] * vec[2]),
            (mat[1][0] * vec[0]) + (mat[1][1] * vec[1]) + (mat[1][2] * vec[2]),
            (mat[2][0] * vec[0]) + (mat[2][1] * vec[1]) + (mat[2][2] * vec[2]),
        ]

    @staticmethod
    def add_vec3(a, b):
        return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]

    @staticmethod
    def normalize_vec3(vec):
        mag = math.sqrt((vec[0] * vec[0]) + (vec[1] * vec[1]) + (vec[2] * vec[2]))
        if mag <= 1e-8:
            return [1.0, 0.0, 0.0]
        return [vec[0] / mag, vec[1] / mag, vec[2] / mag]


def main(args=None):
    rclpy.init(args=args)
    node = RaycastedApproachCoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
