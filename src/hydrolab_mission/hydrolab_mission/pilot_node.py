import json
import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
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
from std_msgs.msg import Float32
from std_msgs.msg import String


class PilotNode(Node):
    def __init__(self):
        super().__init__('pilot_node')

        self.declare_parameter('max_altitude_m', 3.0)
        self.declare_parameter('max_speed_m_s', 1.0)
        self.declare_parameter('hover_contract_topic', '/mission/hover_contract')
        self.declare_parameter('approach_contract_topic', '/mission/approach_contract')
        self.declare_parameter('raycast_distance_topic', '/pilot/raycast_distance_m')
        self.declare_parameter('ibvs_velocity_topic', '/pilot/ibvs_velocity_cmd')
        self.declare_parameter('ibvs_cmd_timeout_s', 0.5)
        self.declare_parameter('ibvs_max_speed_m_s', 0.8)
        self.declare_parameter('ball_hover_error_gain', 0.18)
        self.declare_parameter('ball_hover_deadband', 0.05)
        self.declare_parameter('descent_min_speed_m_s', 0.2)
        self.declare_parameter('raycast_distance_timeout_s', 0.5)

        self.max_altitude_m = float(self.get_parameter('max_altitude_m').value)
        self.max_speed_m_s = max(
            float(self.get_parameter('max_speed_m_s').value),
            0.1,
        )
        self.hover_contract_topic = str(
            self.get_parameter('hover_contract_topic').value
        )
        self.approach_contract_topic = str(
            self.get_parameter('approach_contract_topic').value
        )
        self.raycast_distance_topic = str(
            self.get_parameter('raycast_distance_topic').value
        )
        self.ibvs_velocity_topic = str(
            self.get_parameter('ibvs_velocity_topic').value
        )
        self.ibvs_cmd_timeout_s = max(
            float(self.get_parameter('ibvs_cmd_timeout_s').value),
            0.05,
        )
        self.ibvs_max_speed_m_s = max(
            float(self.get_parameter('ibvs_max_speed_m_s').value),
            0.05,
        )
        self.ball_hover_error_gain = max(
            float(self.get_parameter('ball_hover_error_gain').value),
            0.01,
        )
        self.ball_hover_deadband = max(
            float(self.get_parameter('ball_hover_deadband').value),
            0.0,
        )
        self.descent_min_speed_m_s = max(
            float(self.get_parameter('descent_min_speed_m_s').value),
            0.05,
        )
        self.descent_min_speed_m_s = min(
            self.descent_min_speed_m_s,
            self.max_speed_m_s,
        )
        self.raycast_distance_timeout_s = max(
            float(self.get_parameter('raycast_distance_timeout_s').value),
            0.05,
        )
        self.control_period_s = 0.1

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
        self.hover_contract_subscriber = self.create_subscription(
            String,
            self.hover_contract_topic,
            self.hover_contract_callback,
            10,
        )
        self.approach_contract_subscriber = self.create_subscription(
            String,
            self.approach_contract_topic,
            self.approach_contract_callback,
            10,
        )
        self.raycast_distance_subscriber = self.create_subscription(
            Float32,
            self.raycast_distance_topic,
            self.raycast_distance_callback,
            10,
        )
        self.ibvs_velocity_subscriber = self.create_subscription(
            Vector3,
            self.ibvs_velocity_topic,
            self.ibvs_velocity_callback,
            10,
        )

        self.current_target = [0.0, 0.0, 0.0]
        self.requested_target = None
        self.home_position = None
        self.current_yaw = 0.0
        self.heartbeat_counter = 0
        self.vehicle_connected = False
        self.ball_mode = 'idle'
        self.last_ball_log_ns = 0
        self.last_hover_log_ns = 0
        self.hover_contract_active = False
        self.hover_tracking_mode = 'idle'
        self.approach_contract_active = False
        self.approach_tracking_mode = 'idle'
        self.approach_speed_limit_m_s = None
        self.approach_slowdown_raycast_distance_m = None
        self.approach_final_speed_raycast_distance_m = None
        self.last_valid_approach_speed_limit_m_s = None
        self.latest_raycast_distance_m = None
        self.last_raycast_distance_time_s = 0.0
        self.ibvs_velocity_cmd = [0.0, 0.0, 0.0]
        self.last_ibvs_cmd_time_s = 0.0

        self.timer = self.create_timer(
            self.control_period_s,
            self.timer_callback,
        )
        self.get_logger().info(
            f'Pilot Node initialized. max_altitude={self.max_altitude_m:.2f} m, '
            f'max_speed={self.max_speed_m_s:.2f} m/s, '
            f'ibvs_topic={self.ibvs_velocity_topic}'
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

    def hover_contract_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Received invalid hover contract JSON payload.',
                throttle_duration_sec=2.0,
            )
            return

        if not isinstance(payload, dict):
            self.get_logger().warn(
                'Ignoring hover contract payload with non-object root.',
                throttle_duration_sec=2.0,
            )
            return

        self.hover_contract_active = bool(payload.get('active', False))
        self.hover_tracking_mode = str(
            payload.get('tracking_mode', 'idle')
        ).strip().lower()

    def approach_contract_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Received invalid approach contract JSON payload.',
                throttle_duration_sec=2.0,
            )
            return

        if not isinstance(payload, dict):
            self.get_logger().warn(
                'Ignoring approach contract payload with non-object root.',
                throttle_duration_sec=2.0,
            )
            return

        self.approach_contract_active = bool(payload.get('active', False))
        self.approach_tracking_mode = str(
            payload.get('tracking_mode', 'idle')
        ).strip().lower()
        if not self.approach_contract_active:
            self.last_valid_approach_speed_limit_m_s = None
        constraints = payload.get('constraints', {})
        if not isinstance(constraints, dict):
            constraints = {}
        speed_value = constraints.get('final_speed_m_s', None)
        if speed_value is None:
            self.approach_speed_limit_m_s = None
        else:
            try:
                self.approach_speed_limit_m_s = max(0.05, float(speed_value))
            except (TypeError, ValueError):
                self.approach_speed_limit_m_s = None
        slowdown_distance_value = constraints.get('slowdown_raycast_distance_m', None)
        if slowdown_distance_value is None:
            self.approach_slowdown_raycast_distance_m = None
        else:
            try:
                self.approach_slowdown_raycast_distance_m = max(
                    0.0,
                    float(slowdown_distance_value),
                )
            except (TypeError, ValueError):
                self.approach_slowdown_raycast_distance_m = None
        final_speed_distance_value = constraints.get('final_speed_raycast_distance_m', None)
        if final_speed_distance_value is None:
            self.approach_final_speed_raycast_distance_m = None
        else:
            try:
                self.approach_final_speed_raycast_distance_m = max(
                    0.0,
                    float(final_speed_distance_value),
                )
            except (TypeError, ValueError):
                self.approach_final_speed_raycast_distance_m = None

    def raycast_distance_callback(self, msg):
        distance_m = float(msg.data)
        if not math.isfinite(distance_m) or distance_m < 0.0:
            return
        self.latest_raycast_distance_m = distance_m
        self.last_raycast_distance_time_s = self.get_clock().now().nanoseconds / 1e9

    def ibvs_velocity_callback(self, msg):
        self.ibvs_velocity_cmd = [
            self.clamp(float(msg.x), -self.ibvs_max_speed_m_s, self.ibvs_max_speed_m_s),
            self.clamp(float(msg.y), -self.ibvs_max_speed_m_s, self.ibvs_max_speed_m_s),
            self.clamp(float(msg.z), -self.ibvs_max_speed_m_s, self.ibvs_max_speed_m_s),
        ]
        self.last_ibvs_cmd_time_s = self.get_clock().now().nanoseconds / 1e9

    def request_hover_over_ball(self, detection, color_name):
        if self.home_position is None:
            return

        center_uv = detection.get('center_uv', [0.5, 0.5])
        if len(center_uv) != 2:
            return

        error_u = 0.5 - float(center_uv[0])
        error_v = 0.5 - float(center_uv[1])

        if abs(error_u) < self.ball_hover_deadband:
            error_u = 0.0
        if abs(error_v) < self.ball_hover_deadband:
            error_v = 0.0

        if self.requested_target is None:
            self.requested_target = self.home_position.copy()

        hover_target = self.requested_target.copy()
        hover_target[0] += error_u * self.ball_hover_error_gain
        hover_target[1] += error_v * self.ball_hover_error_gain
        hover_target[2] = self.requested_target[2]
        hover_target = self._sanitize_target(hover_target)
        self.requested_target = hover_target

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_hover_log_ns >= int(1e9):
            self.get_logger().info(
                f'Hovering over {color_name} ball: '
                f'center_uv=({center_uv[0]:.3f},{center_uv[1]:.3f}), '
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
        if self.vehicle_connected:
            return

        self.vehicle_connected = True
        self.home_position = [
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        ]
        self.current_target = self.home_position.copy()

        q = msg.q
        self.current_yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]),
        )

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

        if self.vehicle_connected and not self.is_hover_velocity_mode_active():
            self.update_position_setpoint()

        self.publish_setpoint()

        if self.vehicle_connected:
            if self.heartbeat_counter == 10:
                self.engage_offboard_mode()
                self.arm_drone()

            self.heartbeat_counter += 1

    def publish_control_mode(self):
        msg = OffboardControlMode()
        hover_velocity_mode = self.is_hover_velocity_mode_active()
        msg.position = not hover_velocity_mode
        msg.velocity = hover_velocity_mode
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
        msg.speed_up = self.max_speed_m_s
        msg.speed_down = self.max_speed_m_s
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

        if sanitized_target[2] > self.current_target[2]:
            self.current_target = self.compute_next_descent_setpoint(sanitized_target)
            return

        delta = [sanitized_target[i] - self.current_target[i] for i in range(3)]
        distance = math.sqrt(sum(component * component for component in delta))
        max_step = self.get_position_speed_limit_m_s(distance) * self.control_period_s

        if distance <= max_step or distance == 0.0:
            self.current_target = sanitized_target.copy()
            return

        scale = max_step / distance
        self.current_target = [
            self.current_target[i] + delta[i] * scale
            for i in range(3)
        ]

    def compute_next_descent_setpoint(self, requested_target):
        next_target = self.current_target.copy()
        horizontal_delta_x = requested_target[0] - self.current_target[0]
        horizontal_delta_y = requested_target[1] - self.current_target[1]
        horizontal_distance = math.sqrt(
            (horizontal_delta_x * horizontal_delta_x)
            + (horizontal_delta_y * horizontal_delta_y)
        )

        max_horizontal_step = self.get_position_speed_limit_m_s(horizontal_distance) * self.control_period_s
        if horizontal_distance <= max_horizontal_step or horizontal_distance == 0.0:
            next_target[0] = requested_target[0]
            next_target[1] = requested_target[1]
        else:
            horizontal_scale = max_horizontal_step / horizontal_distance
            next_target[0] = self.current_target[0] + horizontal_delta_x * horizontal_scale
            next_target[1] = self.current_target[1] + horizontal_delta_y * horizontal_scale

        current_height_m = max(0.0, self.home_position[2] - self.current_target[2])
        descent_ratio = 0.0
        if self.max_altitude_m > 1e-6:
            descent_ratio = self.clamp(
                current_height_m / self.max_altitude_m,
                0.0,
                1.0,
            )
        descent_speed_m_s = self.descent_min_speed_m_s + (
            (self.max_speed_m_s - self.descent_min_speed_m_s) * descent_ratio
        )
        max_descent_step = descent_speed_m_s * self.control_period_s

        vertical_delta = requested_target[2] - self.current_target[2]
        next_target[2] = self.current_target[2] + min(vertical_delta, max_descent_step)
        return next_target

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        if self.is_hover_velocity_mode_active():
            velocity_cmd = self.get_active_ibvs_velocity()
            msg.position = [math.nan, math.nan, math.nan]
            msg.velocity = velocity_cmd
            self.get_logger().info(
                'Publishing velocity setpoint (hover mode): '
                f'vx={velocity_cmd[0]:.2f}, vy={velocity_cmd[1]:.2f}, vz={velocity_cmd[2]:.2f}',
                throttle_duration_sec=1.0,
            )
        else:
            msg.position = self.current_target
            msg.velocity = [math.nan, math.nan, math.nan]
            self.get_logger().info(
                'Publishing position setpoint (velocity omitted in position mode): '
                f'x={self.current_target[0]:.2f}, y={self.current_target[1]:.2f}, '
                f'z={self.current_target[2]:.2f}',
                throttle_duration_sec=1.0,
            )
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

    def is_hover_velocity_mode_active(self):
        return self.hover_contract_active and self.hover_tracking_mode == 'hover'

    def get_position_speed_limit_m_s(self, distance_to_target_m):
        speed_limits = []
        debug_reason = 'max_speed_default'
        if (
            self.approach_contract_active
            and self.approach_tracking_mode == 'initial_approach'
            and self.approach_speed_limit_m_s is not None
        ):
            now_s = self.get_clock().now().nanoseconds / 1e9
            has_recent_raycast_distance = (
                self.latest_raycast_distance_m is not None
                and self.last_raycast_distance_time_s > 0.0
                and (now_s - self.last_raycast_distance_time_s) <= self.raycast_distance_timeout_s
            )
            if not has_recent_raycast_distance:
                fallback_limit_m_s = self.last_valid_approach_speed_limit_m_s
                if fallback_limit_m_s is None:
                    fallback_limit_m_s = self.approach_speed_limit_m_s
                if fallback_limit_m_s is not None:
                    speed_limits.append(fallback_limit_m_s)
                    debug_reason = 'raycast_stale_fallback_speed'
                else:
                    debug_reason = 'raycast_stale_or_missing'
            if (
                has_recent_raycast_distance
                and self.approach_slowdown_raycast_distance_m is not None
                and self.approach_slowdown_raycast_distance_m > 1e-6
                and self.latest_raycast_distance_m <= self.approach_slowdown_raycast_distance_m
            ):
                final_speed_distance_m = self.approach_final_speed_raycast_distance_m
                if final_speed_distance_m is None:
                    final_speed_distance_m = 0.0
                final_speed_distance_m = min(
                    final_speed_distance_m,
                    self.approach_slowdown_raycast_distance_m,
                )
                if self.latest_raycast_distance_m <= final_speed_distance_m:
                    adaptive_limit_m_s = self.approach_speed_limit_m_s
                    debug_reason = 'inside_final_speed_zone'
                elif (
                    self.approach_slowdown_raycast_distance_m
                    - final_speed_distance_m
                    <= 1e-6
                ):
                    adaptive_limit_m_s = self.approach_speed_limit_m_s
                    debug_reason = 'degenerate_slowdown_window'
                else:
                    slowdown_ratio = self.clamp(
                        (
                            self.latest_raycast_distance_m - final_speed_distance_m
                        )
                        / (
                            self.approach_slowdown_raycast_distance_m
                            - final_speed_distance_m
                        ),
                        0.0,
                        1.0,
                    )
                    adaptive_limit_m_s = self.approach_speed_limit_m_s + (
                        (self.max_speed_m_s - self.approach_speed_limit_m_s)
                        * slowdown_ratio
                    )
                    debug_reason = 'interpolated_slowdown_zone'
                speed_limits.append(adaptive_limit_m_s)
                self.last_valid_approach_speed_limit_m_s = adaptive_limit_m_s
            elif has_recent_raycast_distance:
                debug_reason = 'outside_slowdown_zone'

        speed_limit_m_s = self.max_speed_m_s
        if speed_limits:
            speed_limit_m_s = min(self.max_speed_m_s, min(speed_limits))

        if self.approach_contract_active and self.approach_tracking_mode == 'initial_approach':
            raycast_text = (
                f'{self.latest_raycast_distance_m:.2f}'
                if self.latest_raycast_distance_m is not None
                else 'n/a'
            )
            self.get_logger().info(
                'Approach speed limiter: '
                f'reason={debug_reason}, '
                f'raycast_distance_m={raycast_text}, '
                f'speed_limit_m_s={speed_limit_m_s:.2f}, '
                f'requested_delta_distance_m={distance_to_target_m:.2f}',
                throttle_duration_sec=1.0,
            )

        return speed_limit_m_s

    def get_active_ibvs_velocity(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        if (
            self.last_ibvs_cmd_time_s > 0.0
            and (now_s - self.last_ibvs_cmd_time_s) <= self.ibvs_cmd_timeout_s
        ):
            return self.ibvs_velocity_cmd
        return [0.0, 0.0, 0.0]

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

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
