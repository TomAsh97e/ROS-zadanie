import json

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HoverControllerNode(Node):
    def __init__(self):
        super().__init__('hover_controller')

        self.declare_parameter('control_interval_s', 0.1)
        self.declare_parameter('hover_contract_topic', '/mission/hover_contract')
        self.declare_parameter('detection_topic', '/vision/ball_detection/result')
        self.declare_parameter('velocity_topic', '/pilot/ibvs_velocity_cmd')
        self.declare_parameter('detection_timeout_s', 0.6)
        self.declare_parameter('enabled_at_start', True)
        self.declare_parameter('ibvs_gain_xy', 1.2)
        self.declare_parameter('ibvs_ki_xy', 0.0)
        self.declare_parameter('ibvs_kd_xy', 0.0)
        self.declare_parameter('ibvs_deadband_uv', 0.02)
        self.declare_parameter('ibvs_max_speed_m_s', 0.8)
        self.declare_parameter('ibvs_derivative_lpf_alpha', 1.0)
        self.declare_parameter('ibvs_max_integral_error', 0.4)

        self.control_interval_s = max(
            float(self.get_parameter('control_interval_s').value),
            0.02,
        )
        self.detection_timeout_s = max(
            float(self.get_parameter('detection_timeout_s').value),
            0.05,
        )
        self.enabled = bool(self.get_parameter('enabled_at_start').value)
        self.ibvs_gain_xy = max(
            float(self.get_parameter('ibvs_gain_xy').value),
            0.01,
        )
        self.ibvs_ki_xy = max(
            float(self.get_parameter('ibvs_ki_xy').value),
            0.0,
        )
        self.ibvs_kd_xy = max(
            float(self.get_parameter('ibvs_kd_xy').value),
            0.0,
        )
        self.ibvs_deadband_uv = max(
            float(self.get_parameter('ibvs_deadband_uv').value),
            0.0,
        )
        self.ibvs_max_speed_m_s = max(
            float(self.get_parameter('ibvs_max_speed_m_s').value),
            0.05,
        )
        self.ibvs_derivative_lpf_alpha = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('ibvs_derivative_lpf_alpha').value),
            ),
        )
        self.ibvs_max_integral_error = max(
            float(self.get_parameter('ibvs_max_integral_error').value),
            0.0,
        )

        hover_contract_topic = str(
            self.get_parameter('hover_contract_topic').value
        )
        detection_topic = str(self.get_parameter('detection_topic').value)
        velocity_topic = str(self.get_parameter('velocity_topic').value)

        self.velocity_publisher = self.create_publisher(Vector3, velocity_topic, 10)
        self.create_subscription(
            String,
            hover_contract_topic,
            self.hover_contract_callback,
            10,
        )
        self.create_subscription(
            String,
            detection_topic,
            self.detection_callback,
            10,
        )

        self.contract_active = False
        self.contract_tracking_mode = 'idle'
        self.contract_target_label = ''
        self.contract_target_variant = ''
        self.contract_target_source = ''
        self.target_center_uv = None
        self.last_detection_time_s = 0.0
        self.last_log_ns = 0
        self.last_control_time_s = 0.0
        self.integral_u = 0.0
        self.integral_v = 0.0
        self.prev_error_u = 0.0
        self.prev_error_v = 0.0
        self.filtered_du = 0.0
        self.filtered_dv = 0.0

        self.timer = self.create_timer(self.control_interval_s, self.timer_callback)
        self.get_logger().info(
            'Hover controller initialized. '
            f'contract_topic={hover_contract_topic}, detection_topic={detection_topic}, '
            f'velocity_topic={velocity_topic}'
        )

    def hover_contract_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Ignoring invalid hover contract payload.',
                throttle_duration_sec=2.0,
            )
            return

        if not isinstance(payload, dict):
            self.get_logger().warn(
                'Ignoring non-object hover contract payload.',
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

        self.contract_target_label = str(target.get('label', '')).strip().lower()
        self.contract_target_variant = str(
            target.get('variant', '')
        ).strip().lower()
        self.contract_target_source = str(
            target.get('source', '')
        ).strip().lower()

    def detection_callback(self, msg):
        center_uv = self.extract_target_center_uv(
            payload_raw=msg.data,
            target_label=self.contract_target_label,
            target_variant=self.contract_target_variant,
        )
        if center_uv is None:
            self.target_center_uv = None
            return

        self.target_center_uv = center_uv
        self.last_detection_time_s = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        if not self.enabled:
            self.reset_pid_state()
            return

        if not self.contract_active:
            self.reset_pid_state()
            self.publish_velocity_command(0.0, 0.0, 0.0)
            return

        if self.contract_tracking_mode != 'hover':
            self.reset_pid_state()
            self.publish_velocity_command(0.0, 0.0, 0.0)
            return

        if (
            self.contract_target_source
            and self.contract_target_source != 'vision/ball_detector'
        ):
            self.reset_pid_state()
            self.publish_velocity_command(0.0, 0.0, 0.0)
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.target_center_uv is None:
            self.reset_pid_state()
            self.publish_velocity_command(0.0, 0.0, 0.0)
            return
        if (now_s - self.last_detection_time_s) > self.detection_timeout_s:
            self.reset_pid_state()
            self.publish_velocity_command(0.0, 0.0, 0.0)
            return

        dt = now_s - self.last_control_time_s
        if self.last_control_time_s <= 0.0 or dt <= 0.0:
            dt = self.control_interval_s
        self.last_control_time_s = now_s

        error_u = 0.5 - float(self.target_center_uv[0])
        error_v = 0.5 - float(self.target_center_uv[1])
        if abs(error_u) < self.ibvs_deadband_uv:
            error_u = 0.0
        if abs(error_v) < self.ibvs_deadband_uv:
            error_v = 0.0

        self.integral_u += error_u * dt
        self.integral_v += error_v * dt
        self.integral_u = self.clamp(
            self.integral_u,
            -self.ibvs_max_integral_error,
            self.ibvs_max_integral_error,
        )
        self.integral_v = self.clamp(
            self.integral_v,
            -self.ibvs_max_integral_error,
            self.ibvs_max_integral_error,
        )

        raw_du = (error_u - self.prev_error_u) / dt
        raw_dv = (error_v - self.prev_error_v) / dt
        alpha = self.ibvs_derivative_lpf_alpha
        self.filtered_du = (alpha * raw_du) + ((1.0 - alpha) * self.filtered_du)
        self.filtered_dv = (alpha * raw_dv) + ((1.0 - alpha) * self.filtered_dv)

        cmd_u = (
            self.ibvs_gain_xy * error_u
            + self.ibvs_ki_xy * self.integral_u
            + self.ibvs_kd_xy * self.filtered_du
        )
        cmd_v = (
            self.ibvs_gain_xy * error_v
            + self.ibvs_ki_xy * self.integral_v
            + self.ibvs_kd_xy * self.filtered_dv
        )

        vx = self.clamp(cmd_u, -self.ibvs_max_speed_m_s, self.ibvs_max_speed_m_s)
        vy = self.clamp(cmd_v, -self.ibvs_max_speed_m_s, self.ibvs_max_speed_m_s)
        vz = 0.0
        self.publish_velocity_command(vx, vy, vz)
        self.prev_error_u = error_u
        self.prev_error_v = error_v

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= int(1e9):
            self.get_logger().info(
                'Hover controller active: '
                f'target={self.contract_target_label}:{self.contract_target_variant}, '
                f'error_uv=({error_u:.3f}, {error_v:.3f}), '
                f'vel=({vx:.2f}, {vy:.2f}, {vz:.2f})'
            )
            self.last_log_ns = now_ns

    def publish_velocity_command(self, vx, vy, vz):
        msg = Vector3()
        msg.x = float(vx)
        msg.y = float(vy)
        msg.z = float(vz)
        self.velocity_publisher.publish(msg)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def reset_pid_state(self):
        self.last_control_time_s = 0.0
        self.integral_u = 0.0
        self.integral_v = 0.0
        self.prev_error_u = 0.0
        self.prev_error_v = 0.0
        self.filtered_du = 0.0
        self.filtered_dv = 0.0

    @staticmethod
    def extract_target_center_uv(payload_raw, target_label='', target_variant=''):
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

            target = detection
            break

        if target is None:
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


def main(args=None):
    rclpy.init(args=args)
    node = HoverControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
