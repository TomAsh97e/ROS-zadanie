import json
import math

from geometry_msgs.msg import Point, Vector3
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleConstraints,
    VehicleOdometry,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class PilotNode(Node):
    def __init__(self):
        super().__init__('pilot_node')

        self.declare_parameter('max_altitude_m', 3.0)
        self.declare_parameter('max_speed_m_s', 1.0)
        self.declare_parameter('transit_speed_m_s', 1.0)
        self.declare_parameter('control_period_s', 0.1)
        self.declare_parameter('ball_hover_error_gain', 0.06)
        self.declare_parameter('ball_hover_deadband', 0.04)
        self.declare_parameter('ball_hover_max_step_m', 0.12)
        self.declare_parameter('ball_hover_step_scale_min', 0.35)
        self.declare_parameter('ball_hover_error_boost_max', 1.0)
        self.declare_parameter('ball_hover_error_boost_reference', 0.25)
        self.declare_parameter('gimbal_step_scale_reference_deg', 35.0)
        self.declare_parameter('gimbal_down_pitch_deg', -90.0)
        self.declare_parameter('gimbal_down_yaw_deg', 0.0)
        self.declare_parameter('gimbal_guidance_gain_m_per_deg', 0.005)
        self.declare_parameter('gimbal_guidance_deadband_deg', 5.0)

        self.max_altitude_m = float(self.get_parameter('max_altitude_m').value)
        self.max_speed_m_s = max(
            float(self.get_parameter('max_speed_m_s').value),
            0.1,
        )
        self.transit_speed_m_s = max(
            float(self.get_parameter('transit_speed_m_s').value),
            self.max_speed_m_s,
        )
        self.control_period_s = max(
            float(self.get_parameter('control_period_s').value),
            0.02,
        )
        self.ball_hover_error_gain = max(
            float(self.get_parameter('ball_hover_error_gain').value),
            0.01,
        )
        self.ball_hover_deadband = max(
            float(self.get_parameter('ball_hover_deadband').value),
            0.0,
        )
        self.ball_hover_max_step_m = max(
            float(self.get_parameter('ball_hover_max_step_m').value),
            0.02,
        )
        self.ball_hover_step_scale_min = min(
            max(float(self.get_parameter('ball_hover_step_scale_min').value), 0.05),
            1.0,
        )
        self.ball_hover_error_boost_max = max(
            float(self.get_parameter('ball_hover_error_boost_max').value),
            1.0,
        )
        self.ball_hover_error_boost_reference = max(
            float(self.get_parameter('ball_hover_error_boost_reference').value),
            0.01,
        )
        self.gimbal_step_scale_reference_deg = max(
            float(self.get_parameter('gimbal_step_scale_reference_deg').value),
            1.0,
        )
        self.gimbal_down_pitch_deg = float(
            self.get_parameter('gimbal_down_pitch_deg').value
        )
        self.gimbal_down_yaw_deg = float(
            self.get_parameter('gimbal_down_yaw_deg').value
        )
        self.gimbal_guidance_gain_m_per_deg = max(
            float(self.get_parameter('gimbal_guidance_gain_m_per_deg').value),
            0.001,
        )
        self.gimbal_guidance_deadband_deg = max(
            float(self.get_parameter('gimbal_guidance_deadband_deg').value),
            0.0,
        )

        self.mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        self.command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )
        self.constraints_publisher = self.create_publisher(
            VehicleConstraints,
            '/fmu/in/vehicle_constraints',
            10,
        )

        self.target_subscriber = self.create_subscription(
            Point,
            '/pilot/target_position',
            self.target_callback,
            10,
        )
        self.odom_subscriber = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile_sensor_data,
        )
        self.ball_mode_subscriber = self.create_subscription(
            String,
            '/pilot/ball_mode',
            self.ball_mode_callback,
            10,
        )
        self.ball_detection_subscriber = self.create_subscription(
            String,
            '/vision/ball_detection/result',
            self.ball_detection_callback,
            10,
        )
        self.gimbal_attitude_subscriber = self.create_subscription(
            Vector3,
            '/gimbal/current_orientation_deg',
            self.gimbal_attitude_callback,
            10,
        )

        self.current_target = [0.0, 0.0, 0.0]
        self.requested_target = None
        self.home_position = None
        self.current_position = None
        self.current_yaw = 0.0
        self.current_gimbal_pitch_deg = self.gimbal_down_pitch_deg
        self.current_gimbal_yaw_deg = self.gimbal_down_yaw_deg
        self.has_gimbal_attitude = False
        self.heartbeat_counter = 0
        self.vehicle_connected = False
        self.ball_mode = 'idle'
        self.last_ball_log_ns = 0
        self.last_hover_log_ns = 0

        self.timer = self.create_timer(
            self.control_period_s,
            self.timer_callback,
        )
        self.get_logger().info(
            f'Pilot Node initialized. max_altitude={self.max_altitude_m:.2f} m, '
            f'precision_speed={self.max_speed_m_s:.2f} m/s, '
            f'transit_speed={self.transit_speed_m_s:.2f} m/s'
        )

    def ball_mode_callback(self, msg):
        mode = msg.data.strip().lower()
        if mode in ('', 'stop', 'off', 'disable', 'none'):
            self.ball_mode = 'idle'
            self.get_logger().info('Pilot ball mode disabled.')
            return

        self.ball_mode = mode
        self.get_logger().info(f'Pilot ball mode set to: {mode}')

    def ball_detection_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Received invalid ball detection JSON payload.')
            return

        if not payload.get('active', False):
            return

        detections = payload.get('detections', [])
        if not detections:
            return

        if self.ball_mode.startswith('hover_'):
            target_color = self.ball_mode.split('hover_', 1)[1]
            target_detection = next(
                (item for item in detections if item.get('color') == target_color),
                None,
            )
            if target_detection is not None:
                self.request_hover_over_ball(target_detection, target_color)

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_ball_log_ns < int(1e9):
            return

        summary = ', '.join(
            (
                f'{item.get("color", "unknown")}'
                f'@({item.get("center_px", ["?", "?"])[0]},'
                f'{item.get("center_px", ["?", "?"])[1]})'
            )
            for item in detections
        )
        self.get_logger().info(f'Pilot sees balls: {summary}')
        self.last_ball_log_ns = now_ns

    def gimbal_attitude_callback(self, msg):
        self.current_gimbal_pitch_deg = float(msg.y)
        self.current_gimbal_yaw_deg = float(msg.z)
        self.has_gimbal_attitude = True

    def request_hover_over_ball(self, detection, color_name):
        if self.current_position is None:
            return

        center_uv = detection.get('center_uv', [0.5, 0.5])
        if len(center_uv) != 2:
            return

        image_right_error = float(center_uv[0]) - 0.5
        image_down_error = float(center_uv[1]) - 0.5
        image_error_magnitude = math.hypot(image_right_error, image_down_error)
        forward_step = 0.0
        right_step = 0.0

        off_down_deg = 0.0
        if self.has_gimbal_attitude:
            off_down_deg = max(
                self.current_gimbal_pitch_deg - self.gimbal_down_pitch_deg,
                0.0,
            )

        step_scale_ratio = min(
            off_down_deg / self.gimbal_step_scale_reference_deg,
            1.0,
        )
        step_scale = (
            self.ball_hover_step_scale_min
            + (1.0 - self.ball_hover_step_scale_min) * step_scale_ratio
        )
        error_boost_ratio = min(
            image_error_magnitude / self.ball_hover_error_boost_reference,
            1.0,
        )
        error_boost = (
            1.0
            + (self.ball_hover_error_boost_max - 1.0) * error_boost_ratio
        )
        effective_hover_gain = self.ball_hover_error_gain * step_scale * error_boost
        effective_max_step_m = max(
            self.ball_hover_max_step_m * step_scale * error_boost,
            0.02,
        )

        if self.has_gimbal_attitude:
            if off_down_deg > self.gimbal_guidance_deadband_deg:
                step_magnitude = (
                    off_down_deg * self.gimbal_guidance_gain_m_per_deg
                )
                bearing_rad = math.radians(
                    self.current_gimbal_yaw_deg - self.gimbal_down_yaw_deg
                )
                forward_step += math.cos(bearing_rad) * step_magnitude
                right_step += math.sin(bearing_rad) * step_magnitude

        if abs(image_right_error) < self.ball_hover_deadband:
            image_right_correction = 0.0
        else:
            image_right_correction = image_right_error * effective_hover_gain
        if abs(image_down_error) < self.ball_hover_deadband:
            image_forward_correction = 0.0
        else:
            image_forward_correction = -image_down_error * effective_hover_gain

        forward_step += image_forward_correction
        right_step += image_right_correction

        step_magnitude = math.hypot(forward_step, right_step)
        if step_magnitude > effective_max_step_m:
            scale = effective_max_step_m / step_magnitude
            forward_step *= scale
            right_step *= scale

        yaw = self.current_yaw
        world_x_step = (
            math.cos(yaw) * forward_step
            - math.sin(yaw) * right_step
        )
        world_y_step = (
            math.sin(yaw) * forward_step
            + math.cos(yaw) * right_step
        )

        hover_target = self.current_position.copy()
        hover_target[0] += world_x_step
        hover_target[1] += world_y_step
        if self.requested_target is not None:
            hover_target[2] = self.requested_target[2]
        elif self.home_position is not None:
            hover_target[2] = self.home_position[2]
        hover_target = self._sanitize_target(hover_target)
        self.requested_target = hover_target

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_hover_log_ns >= int(1e9):
            self.get_logger().info(
                f'Hovering over {color_name} ball: '
                f'center_uv=({center_uv[0]:.3f},{center_uv[1]:.3f}), '
                f'gimbal=(pitch={self.current_gimbal_pitch_deg:.1f}, yaw={self.current_gimbal_yaw_deg:.1f}, off_down={off_down_deg:.1f}), '
                f'step_scale={step_scale:.2f}, '
                f'error_boost={error_boost:.2f}, '
                f'world_step=({world_x_step:.2f}, {world_y_step:.2f}), '
                f'target=({hover_target[0]:.2f}, {hover_target[1]:.2f}, {hover_target[2]:.2f})'
            )
            self.last_hover_log_ns = now_ns

    def target_callback(self, msg):
        requested_target = [msg.x, msg.y, msg.z]
        clamped_target = self._sanitize_target(requested_target)
        self.requested_target = clamped_target

        if not math.isclose(clamped_target[2], requested_target[2], abs_tol=1e-6):
            self.get_logger().warn(
                'Target altitude clamped from '
                f'{requested_target[2]:.2f} to {clamped_target[2]:.2f}'
            )

        self.get_logger().info(
            f'Received new target: X={clamped_target[0]:.2f}, '
            f'Y={clamped_target[1]:.2f}, Z={clamped_target[2]:.2f}'
        )

    def odometry_callback(self, msg):
        self.current_position = [
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        ]

        q = msg.q
        self.current_yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]),
        )

        if self.vehicle_connected:
            return

        self.vehicle_connected = True
        self.home_position = self.current_position.copy()
        self.current_target = self.home_position.copy()

        if self.requested_target is None:
            self.requested_target = self.home_position.copy()
        else:
            self.requested_target = self._sanitize_target(self.requested_target)

        self.get_logger().info('✅ Connection established! Telemetry received.')
        self.get_logger().info(
            f'🔒 Locked initial yaw at {self.current_yaw:.2f} rad and home at '
            f'({self.home_position[0]:.2f}, {self.home_position[1]:.2f}, '
            f'{self.home_position[2]:.2f})'
        )

    def timer_callback(self):
        self.publish_control_mode()
        self.publish_vehicle_constraints()

        if self.vehicle_connected:
            self.update_position_setpoint()

        self.publish_setpoint()

        if self.vehicle_connected:
            if self.heartbeat_counter == 10:
                self.engage_offboard_mode()
                self.arm_drone()

            self.heartbeat_counter += 1

    def publish_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = self.timestamp_us()
        self.mode_publisher.publish(msg)

    def publish_vehicle_constraints(self):
        msg = VehicleConstraints()
        msg.timestamp = self.timestamp_us()
        speed_limit_m_s = self.active_speed_limit_m_s()
        msg.speed_up = speed_limit_m_s
        msg.speed_down = speed_limit_m_s
        msg.want_takeoff = bool(
            self.home_position is not None
            and self.requested_target is not None
            and self.requested_target[2] < self.home_position[2] - 0.1
        )
        self.constraints_publisher.publish(msg)

    def update_position_setpoint(self):
        if self.requested_target is None:
            return

        sanitized_target = self._sanitize_target(self.requested_target)
        self.requested_target = sanitized_target

        delta = [sanitized_target[i] - self.current_target[i] for i in range(3)]
        distance = math.sqrt(sum(component * component for component in delta))
        max_step = self.active_speed_limit_m_s() * self.control_period_s

        if distance <= max_step or distance == 0.0:
            self.current_target = sanitized_target.copy()
            return

        scale = max_step / distance
        self.current_target = [
            self.current_target[i] + delta[i] * scale
            for i in range(3)
        ]

    def active_speed_limit_m_s(self):
        if self.ball_mode.startswith('hover_'):
            return self.max_speed_m_s
        return self.transit_speed_m_s

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = self.current_target
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = self.current_yaw
        msg.yawspeed = math.nan
        msg.timestamp = self.timestamp_us()
        self.setpoint_publisher.publish(msg)

    def arm_drone(self):
        self.get_logger().info('Arming Drone...')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def engage_offboard_mode(self):
        self.get_logger().info('Switching to Offboard Mode...')
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp_us()
        self.command_publisher.publish(msg)

    def _sanitize_target(self, target):
        sanitized_target = target.copy()

        if self.home_position is not None:
            min_z = self.home_position[2] - self.max_altitude_m
            sanitized_target[2] = max(sanitized_target[2], min_z)

        return sanitized_target

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = PilotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
