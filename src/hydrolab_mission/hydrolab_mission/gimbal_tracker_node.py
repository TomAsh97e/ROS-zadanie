import json

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class GimbalTrackerNode(Node):
    def __init__(self):
        super().__init__('gimbal_tracker')

        self.declare_parameter('camera_hfov_deg', 114.6)
        self.declare_parameter('camera_vfov_deg', 71.6)
        self.declare_parameter('tracking_p_gain', 0.5)
        self.declare_parameter('tracking_i_gain_pitch', 0.0)
        self.declare_parameter('tracking_i_gain_yaw', 0.0)
        self.declare_parameter('tracking_d_gain_pitch', 0.0)
        self.declare_parameter('tracking_d_gain_yaw', 0.0)
        self.declare_parameter('derivative_lpf_alpha', 1.0)
        self.declare_parameter('control_interval_s', 0.1)
        self.declare_parameter('detection_timeout_s', 0.6)
        self.declare_parameter('enabled_at_start', True)
        self.declare_parameter('measurement_lpf_alpha', 1.0)
        self.declare_parameter('max_pitch_rate_deg_s', 120.0)
        self.declare_parameter('max_yaw_rate_deg_s', 180.0)
        self.declare_parameter('max_pitch_i_term_deg', 20.0)
        self.declare_parameter('max_yaw_i_term_deg', 20.0)
        self.declare_parameter('min_yaw_deg', -180.0)
        self.declare_parameter('max_yaw_deg', 180.0)

        self.camera_hfov_deg = float(self.get_parameter('camera_hfov_deg').value)
        self.camera_vfov_deg = float(self.get_parameter('camera_vfov_deg').value)
        self.tracking_p_gain = float(self.get_parameter('tracking_p_gain').value)
        self.tracking_i_gain_pitch = float(
            self.get_parameter('tracking_i_gain_pitch').value
        )
        self.tracking_i_gain_yaw = float(
            self.get_parameter('tracking_i_gain_yaw').value
        )
        self.tracking_d_gain_pitch = float(
            self.get_parameter('tracking_d_gain_pitch').value
        )
        self.tracking_d_gain_yaw = float(
            self.get_parameter('tracking_d_gain_yaw').value
        )
        self.derivative_lpf_alpha = float(
            self.get_parameter('derivative_lpf_alpha').value
        )
        self.derivative_lpf_alpha = max(0.0, min(1.0, self.derivative_lpf_alpha))
        self.control_interval_s = max(
            float(self.get_parameter('control_interval_s').value),
            0.02,
        )
        self.detection_timeout_s = max(
            float(self.get_parameter('detection_timeout_s').value),
            0.05,
        )
        self.tracking_enabled = bool(self.get_parameter('enabled_at_start').value)
        self.measurement_lpf_alpha = float(
            self.get_parameter('measurement_lpf_alpha').value
        )
        self.measurement_lpf_alpha = max(0.0, min(1.0, self.measurement_lpf_alpha))
        self.max_pitch_rate_deg_s = max(
            float(self.get_parameter('max_pitch_rate_deg_s').value),
            1.0,
        )
        self.max_yaw_rate_deg_s = max(
            float(self.get_parameter('max_yaw_rate_deg_s').value),
            1.0,
        )
        self.max_pitch_i_term_deg = max(
            float(self.get_parameter('max_pitch_i_term_deg').value),
            0.0,
        )
        self.max_yaw_i_term_deg = max(
            float(self.get_parameter('max_yaw_i_term_deg').value),
            0.0,
        )
        self.min_pitch_deg = -120.0
        self.max_pitch_deg = 30.0
        self.min_yaw_deg = float(self.get_parameter('min_yaw_deg').value)
        self.max_yaw_deg = float(self.get_parameter('max_yaw_deg').value)
        if self.min_yaw_deg > self.max_yaw_deg:
            self.min_yaw_deg, self.max_yaw_deg = self.max_yaw_deg, self.min_yaw_deg

        self.target_publisher = self.create_publisher(
            Vector3,
            '/gimbal/target_orientation_deg',
            10,
        )
        self.active_publisher = self.create_publisher(
            Bool,
            '/gimbal/tracker/active',
            10,
        )
        self.detection_age_publisher = self.create_publisher(
            Float32,
            '/gimbal/tracker/detection_age_s',
            10,
        )
        self.error_deg_publisher = self.create_publisher(
            Vector3,
            '/gimbal/tracker/error_deg',
            10,
        )

        self.vision_subscription = self.create_subscription(
            String,
            '/vision/ball_detection/result',
            self.vision_callback,
            10,
        )
        self.orientation_subscription = self.create_subscription(
            Vector3,
            '/gimbal/current_orientation_deg',
            self.current_orientation_callback,
            10,
        )
        self.enable_subscription = self.create_subscription(
            Bool,
            '/gimbal/tracker/enable',
            self.enable_callback,
            10,
        )

        self.target_roll_deg = 0.0
        self.target_pitch_deg = 0.0
        self.target_yaw_deg = 0.0
        self.latest_u = 0.5
        self.latest_v = 0.5
        self.filtered_u = 0.5
        self.filtered_v = 0.5
        self.has_measurement = False
        self.new_measurement_available = False
        self.last_detection_time_s = 0.0
        self.last_command_update_time_s = 0.0
        self.last_error_pitch_deg = 0.0
        self.last_error_yaw_deg = 0.0
        self.integral_error_pitch = 0.0
        self.integral_error_yaw = 0.0
        self.prev_pitch_control_error = 0.0
        self.prev_yaw_control_error = 0.0
        self.filtered_d_pitch = 0.0
        self.filtered_d_yaw = 0.0
        self.tracking_active = False
        self.timer = self.create_timer(
            self.control_interval_s,
            self.control_loop_callback,
        )
        self.get_logger().info('Gimbal tracker initialized (P-only).')

    def current_orientation_callback(self, msg):
        if self.tracking_active:
            return
        self.target_roll_deg = float(msg.x)
        self.target_pitch_deg = float(msg.y)
        self.target_yaw_deg = float(msg.z)

    def enable_callback(self, msg):
        self.tracking_enabled = bool(msg.data)
        if not self.tracking_enabled:
            self.tracking_active = False
            self.last_error_pitch_deg = 0.0
            self.last_error_yaw_deg = 0.0
            self.reset_integrator()
            self.reset_derivative()
        state_str = 'ENABLED' if self.tracking_enabled else 'DISABLED'
        self.get_logger().info(f'Gimbal tracker {state_str} via /gimbal/tracker/enable.')

    def vision_callback(self, msg):
        center_uv = self.extract_target_center_uv(msg.data)
        if center_uv is None:
            self.has_measurement = False
            return

        self.latest_u = float(center_uv[0])
        self.latest_v = float(center_uv[1])
        self.has_measurement = True
        self.new_measurement_available = True
        self.last_detection_time_s = self.get_clock().now().nanoseconds / 1e9

    def control_loop_callback(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        detection_fresh = (
            self.has_measurement
            and (now_s - self.last_detection_time_s) <= self.detection_timeout_s
        )
        self.tracking_active = self.tracking_enabled and detection_fresh

        if self.tracking_active and self.new_measurement_available:
            alpha = self.measurement_lpf_alpha
            self.filtered_u = (alpha * self.latest_u) + ((1.0 - alpha) * self.filtered_u)
            self.filtered_v = (alpha * self.latest_v) + ((1.0 - alpha) * self.filtered_v)

            error_yaw = self.filtered_u - 0.5
            error_pitch = self.filtered_v - 0.5
            self.last_error_yaw_deg = error_yaw * self.camera_hfov_deg
            self.last_error_pitch_deg = error_pitch * self.camera_vfov_deg

            if self.last_command_update_time_s <= 0.0:
                dt_cmd = self.control_interval_s
            else:
                dt_cmd = now_s - self.last_command_update_time_s
            if dt_cmd <= 0.0:
                dt_cmd = self.control_interval_s
            self.last_command_update_time_s = now_s

            pitch_control_error = -self.last_error_pitch_deg
            yaw_control_error = self.last_error_yaw_deg

            self.integrate_with_anti_windup(
                dt_cmd=dt_cmd,
                pitch_control_error=pitch_control_error,
                yaw_control_error=yaw_control_error,
            )

            p_pitch_step = pitch_control_error * self.tracking_p_gain
            p_yaw_step = yaw_control_error * self.tracking_p_gain
            i_pitch_step = self.tracking_i_gain_pitch * self.integral_error_pitch
            i_yaw_step = self.tracking_i_gain_yaw * self.integral_error_yaw
            d_pitch_step, d_yaw_step = self.compute_filtered_derivative_steps(
                pitch_control_error=pitch_control_error,
                yaw_control_error=yaw_control_error,
                dt_cmd=dt_cmd,
            )
            pitch_step = p_pitch_step + i_pitch_step + d_pitch_step
            yaw_step = p_yaw_step + i_yaw_step + d_yaw_step
            max_pitch_step = self.max_pitch_rate_deg_s * dt_cmd
            max_yaw_step = self.max_yaw_rate_deg_s * dt_cmd
            pitch_step = max(-max_pitch_step, min(max_pitch_step, pitch_step))
            yaw_step = max(-max_yaw_step, min(max_yaw_step, yaw_step))

            self.target_pitch_deg += pitch_step
            self.target_yaw_deg += yaw_step
            self.target_pitch_deg = max(self.min_pitch_deg, min(self.max_pitch_deg, self.target_pitch_deg))
            self.target_yaw_deg = max(self.min_yaw_deg, min(self.max_yaw_deg, self.target_yaw_deg))
            self.new_measurement_available = False
        elif not self.tracking_active:
            self.last_error_pitch_deg = 0.0
            self.last_error_yaw_deg = 0.0
            self.reset_integrator()
            self.reset_derivative()

        if self.tracking_active:
            target_msg = Vector3()
            target_msg.x = float(self.target_roll_deg)
            target_msg.y = float(self.target_pitch_deg)
            target_msg.z = float(self.target_yaw_deg)
            self.target_publisher.publish(target_msg)

        self.publish_diagnostics()

    def extract_target_center_uv(self, payload_raw):
        try:
            payload = json.loads(payload_raw)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Ignoring invalid ball detection payload from vision node.',
                throttle_duration_sec=2.0,
            )
            return None

        if not isinstance(payload, dict):
            self.get_logger().warn(
                'Ignoring detection payload with non-object root.',
                throttle_duration_sec=2.0,
            )
            return None

        active = payload.get('active', False)
        if not isinstance(active, bool) or not active:
            return None

        detections = payload.get('detections', [])
        if not isinstance(detections, list):
            self.get_logger().warn(
                'Ignoring detection payload: "detections" is not a list.',
                throttle_duration_sec=2.0,
            )
            return None

        requested_color = payload.get('request', '')
        if not isinstance(requested_color, str):
            requested_color = ''

        target = None
        for det in detections:
            if not isinstance(det, dict):
                continue
            color = det.get('color')
            if isinstance(color, str) and (color == requested_color or requested_color == 'all'):
                target = det
                break

        if target is None:
            return None

        center_uv = target.get('center_uv')
        if not isinstance(center_uv, list) or len(center_uv) != 2:
            self.get_logger().warn(
                'Ignoring detection payload: target has invalid "center_uv".',
                throttle_duration_sec=2.0,
            )
            return None

        try:
            u = float(center_uv[0])
            v = float(center_uv[1])
        except (TypeError, ValueError):
            self.get_logger().warn(
                'Ignoring detection payload: target "center_uv" is not numeric.',
                throttle_duration_sec=2.0,
            )
            return None

        if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
            self.get_logger().warn(
                'Ignoring detection payload: target "center_uv" outside [0,1].',
                throttle_duration_sec=2.0,
            )
            return None

        return [u, v]

    def publish_diagnostics(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        age_s = now_s - self.last_detection_time_s if self.last_detection_time_s > 0.0 else -1.0
        self.active_publisher.publish(Bool(data=self.tracking_active))
        self.detection_age_publisher.publish(Float32(data=float(age_s)))

        error_msg = Vector3()
        error_msg.x = 0.0
        error_msg.y = float(self.last_error_pitch_deg)
        error_msg.z = float(self.last_error_yaw_deg)
        self.error_deg_publisher.publish(error_msg)

    def integrate_with_anti_windup(
        self,
        dt_cmd,
        pitch_control_error,
        yaw_control_error,
    ):
        # Freeze integration if command is already on a hard bound and error
        # would push farther into saturation.
        pitch_blocked = (
            (self.target_pitch_deg <= self.min_pitch_deg and pitch_control_error < 0.0)
            or (self.target_pitch_deg >= self.max_pitch_deg and pitch_control_error > 0.0)
        )
        yaw_blocked = (
            (self.target_yaw_deg <= self.min_yaw_deg and yaw_control_error < 0.0)
            or (self.target_yaw_deg >= self.max_yaw_deg and yaw_control_error > 0.0)
        )

        if not pitch_blocked:
            self.integral_error_pitch += pitch_control_error * dt_cmd
        if not yaw_blocked:
            self.integral_error_yaw += yaw_control_error * dt_cmd

        self.integral_error_pitch = self.clamp_integral_from_i_term(
            self.integral_error_pitch,
            self.tracking_i_gain_pitch,
            self.max_pitch_i_term_deg,
        )
        self.integral_error_yaw = self.clamp_integral_from_i_term(
            self.integral_error_yaw,
            self.tracking_i_gain_yaw,
            self.max_yaw_i_term_deg,
        )

    @staticmethod
    def clamp_integral_from_i_term(integral_error, ki, max_i_term):
        if max_i_term <= 0.0 or abs(ki) < 1e-9:
            return 0.0
        max_integral = max_i_term / abs(ki)
        return max(-max_integral, min(max_integral, integral_error))

    def reset_integrator(self):
        self.integral_error_pitch = 0.0
        self.integral_error_yaw = 0.0

    def compute_filtered_derivative_steps(
        self,
        pitch_control_error,
        yaw_control_error,
        dt_cmd,
    ):
        raw_d_pitch = (pitch_control_error - self.prev_pitch_control_error) / dt_cmd
        raw_d_yaw = (yaw_control_error - self.prev_yaw_control_error) / dt_cmd

        alpha = self.derivative_lpf_alpha
        self.filtered_d_pitch = (alpha * raw_d_pitch) + ((1.0 - alpha) * self.filtered_d_pitch)
        self.filtered_d_yaw = (alpha * raw_d_yaw) + ((1.0 - alpha) * self.filtered_d_yaw)

        self.prev_pitch_control_error = pitch_control_error
        self.prev_yaw_control_error = yaw_control_error

        d_pitch_step = self.tracking_d_gain_pitch * self.filtered_d_pitch
        d_yaw_step = self.tracking_d_gain_yaw * self.filtered_d_yaw
        return d_pitch_step, d_yaw_step

    def reset_derivative(self):
        self.prev_pitch_control_error = 0.0
        self.prev_yaw_control_error = 0.0
        self.filtered_d_pitch = 0.0
        self.filtered_d_yaw = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = GimbalTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
