import json

from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand
import rclpy
from rclpy.node import Node


class GimbalControllerNode(Node):
    def __init__(self):
        super().__init__('gimbal_controller')

        self.declare_parameter('source_system_id', 1)
        self.declare_parameter('source_component_id', 191)
        self.declare_parameter('command_interval_s', 0.1)
        
        # Camera configuration and searching sequence
        self.declare_parameter('camera_hfov_deg', 114.6)
        self.declare_parameter('camera_vfov_deg', 71.6)
        self.declare_parameter('tracking_p_gain', 0.35)
        self.declare_parameter('tracking_deadband_u', 0.02)
        self.declare_parameter('tracking_deadband_v', 0.02)
        self.declare_parameter('search_waypoint_duration_s', 2.5)
        self.declare_parameter('command_slew_rate_deg_s', 180.0)
        self.declare_parameter('lock_timeout_s', 2.0)
        self.declare_parameter('search_waypoints', [
            -65.0, 0.0,
            -90.0, 0.0,
            -65.0, 0.0,
            -65.0, -60.0,
            -65.0, -120.0,
            -90.0, -120.0,
            -65.0, -120.0,
            -65.0, -60.0, 
            -65.0, 0.0, 
            -90.0, 0.0,
            -65.0, 0.0,
            -65.0, 60.0, 
            -65.0, 120.0,
            -90.0, 120.0,
            -65.0, 120.0,
            -65.0, 60.0
        ])
        self.source_system_id = int(self.get_parameter('source_system_id').value)
        self.source_component_id = int(self.get_parameter('source_component_id').value)
        self.command_interval_s = max(
            float(self.get_parameter('command_interval_s').value),
            0.02,
        )
        self.camera_hfov_deg = float(self.get_parameter('camera_hfov_deg').value)
        self.camera_vfov_deg = float(self.get_parameter('camera_vfov_deg').value)
        self.tracking_p_gain = float(self.get_parameter('tracking_p_gain').value)
        self.tracking_deadband_u = max(
            float(self.get_parameter('tracking_deadband_u').value),
            0.0,
        )
        self.tracking_deadband_v = max(
            float(self.get_parameter('tracking_deadband_v').value),
            0.0,
        )
        self.search_waypoint_duration_s = max(
            float(self.get_parameter('search_waypoint_duration_s').value),
            0.1,
        )
        self.command_slew_rate_deg_s = max(
            float(self.get_parameter('command_slew_rate_deg_s').value),
            1.0,
        )
        self.lock_timeout_s = max(
            float(self.get_parameter('lock_timeout_s').value),
            0.2,
        )
        
        wp_list = self.get_parameter('search_waypoints').value
        # convert flat list into list of tuples (pitch, yaw)
        self.search_waypoints = [
            (
                float(wp_list[i]),
                self.normalize_angle_deg(float(wp_list[i + 1])),
            )
            for i in range(0, len(wp_list), 2)
        ]


        self.command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )
        self.target_subscription = self.create_subscription(
            Vector3,
            '/gimbal/target_orientation_deg',
            self.target_callback,
            10,
        )

        self.state = 'IDLE'  # IDLE, SEARCHING, LOCKED, HOLD
        self.auto_track_enabled = False
        self.last_target_time_s = 0.0
        
        self.status_publisher = self.create_publisher(String, '/gimbal/status', 10)
        self.attitude_publisher = self.create_publisher(Vector3, '/gimbal/current_orientation_deg', 10)
        
        self.command_subscription = self.create_subscription(
            String,
            '/gimbal/command',
            self.command_callback,
            10,
        )
        
        self.vision_subscription = self.create_subscription(
            String,
            '/vision/ball_detection/result',
            self.vision_callback,
            10,
        )

        self.target_roll_deg = 0.0
        self.target_pitch_deg = 0.0
        self.target_yaw_deg = 0.0
        self.commanded_roll_deg = 0.0
        self.commanded_pitch_deg = 0.0
        self.commanded_yaw_deg = 0.0
        self.has_target = False

        # ==========================================
        # THE DISCRETE SEQUENCE (Pitch, Yaw)
        # ==========================================
        self.current_waypoint_index = 0
        self.time_at_waypoint_s = 0.0
        self.waypoint_duration_s = self.search_waypoint_duration_s

        
        self.last_timer_time = self.get_clock().now()

        self.timer = self.create_timer(
            self.command_interval_s,
            self.timer_callback,
        )
        self.get_logger().info('Gimbal Controller initialized with Discrete Sequence.')

    def target_callback(self, msg):
        self.target_roll_deg = msg.x
        self.target_pitch_deg = msg.y
        self.target_yaw_deg = self.normalize_angle_deg(msg.z)
        self.has_target = True

        if not hasattr(self, 'gimbal_configured'):
            self.configure_gimbal_manager()
            self.gimbal_configured = True

        self.get_logger().info(
            f'Updated gimbal target: roll={self.target_roll_deg:.1f} deg, '
            f'pitch={self.target_pitch_deg:.1f} deg, '
            f'yaw={self.target_yaw_deg:.1f} deg'
        )

    def timer_callback(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        dt = (now - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = now
        
        # If locked but we haven't seen the target in 2 seconds, go back to searching
        if self.state == 'LOCKED' and (now_s - self.last_target_time_s) > self.lock_timeout_s:
            self.state = 'SEARCHING'
            self.get_logger().info(
                'Target lost. Resuming Discrete Sequence Search.'
            )
            self.time_at_waypoint_s = 0.0

        if self.state == 'SEARCHING':
            self.time_at_waypoint_s += dt
            
            # If we've been at this waypoint long enough, move to the next one
            if self.time_at_waypoint_s >= self.waypoint_duration_s:
                self.time_at_waypoint_s = 0.0
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.search_waypoints)
                self.get_logger().info(
                    f'Gimbal Sequence: Moving to step '
                    f'{self.current_waypoint_index + 1}/{len(self.search_waypoints)}'
                )

            # Grab the coordinates for the current step in the sequence
            target_p, target_y = self.search_waypoints[self.current_waypoint_index]
            self.target_pitch_deg = target_p
            self.target_yaw_deg = target_y
            self.has_target = True

        if not self.has_target:
            return

        if not hasattr(self, 'gimbal_configured'):
            self.configure_gimbal_manager()
            self.gimbal_configured = True

        self.update_commanded_orientation(dt)
        self.publish_gimbal_command()
        
        msg = String()
        msg.data = f'STATUS_{self.state}'
        self.status_publisher.publish(msg)

        att_msg = Vector3()
        att_msg.x = float(self.commanded_roll_deg)
        att_msg.y = float(self.commanded_pitch_deg)
        att_msg.z = float(self.commanded_yaw_deg)
        self.attitude_publisher.publish(att_msg)

    def command_callback(self, msg):
        cmd = msg.data.upper()
        if cmd == 'COMMAND_SEARCH':
            self.state = 'SEARCHING'
            self.auto_track_enabled = True
            self.current_waypoint_index = 0
            self.time_at_waypoint_s = 0.0

            self.get_logger().info(
                'Gimbal received command: Entering Discrete Sequence Search.'
            )
        elif cmd == 'COMMAND_HOLD':
            self.state = 'HOLD'
            self.auto_track_enabled = False
            self.get_logger().info(
                'Gimbal received command: Holding current absolute orientation.'
            )
        elif cmd == 'COMMAND_RESET':
            self.state = 'IDLE'
            self.auto_track_enabled = False
            self.target_pitch_deg = 0.0
            self.target_yaw_deg = 0.0
            self.has_target = True
            self.get_logger().info(
                'Gimbal received command: Resetting to IDLE neutral.'
            )

    def vision_callback(self, msg):
        if not self.auto_track_enabled:
            return

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                'Ignoring invalid ball detection payload from vision node.',
                throttle_duration_sec=2.0,
            )
            return

        if not payload.get('active', False):
            return

        detections = payload.get('detections', [])
        requested_color = payload.get('request', '')
        
        target = None
        for det in detections:
            if det.get('color') == requested_color or requested_color == 'all':
                target = det
                break

        if target is not None:
            # We see the ball! Switch to locked mode
            if self.state != 'LOCKED':
                self.get_logger().info(f'Gimbal locked on detection request="{requested_color}".')
            
            self.state = 'LOCKED'
            self.last_target_time_s = self.get_clock().now().nanoseconds / 1e9
            
            # center_uv is [0.0 to 1.0], where 0.5 is center.
            # u is horizontal (left to right), v is vertical (top to bottom)
            u, v = target.get('center_uv', [0.5, 0.5])
            
            # Simple Proportional controller for tracking
            error_yaw = u - 0.5
            error_pitch = v - 0.5
            if abs(error_yaw) < self.tracking_deadband_u:
                error_yaw = 0.0
            if abs(error_pitch) < self.tracking_deadband_v:
                error_pitch = 0.0

            # HFOV and VFOV from config
            # Convert percentage error to degrees, multiply by a proportional gain for smooth following.
            # Positive u error (target is to the right) -> Increase yaw
            # Positive v error (target is lower in image) -> Decrease pitch (pitch down)
            
            self.target_yaw_deg = self.normalize_angle_deg(
                self.commanded_yaw_deg
                + (error_yaw * self.camera_hfov_deg) * self.tracking_p_gain
            )
            self.target_pitch_deg = (
                self.commanded_pitch_deg
                - (error_pitch * self.camera_vfov_deg) * self.tracking_p_gain
            )
            
            # Clamp limits (optional but safe)
            self.target_pitch_deg = max(-90.0, min(30.0, self.target_pitch_deg))
            
            self.has_target = True

    def configure_gimbal_manager(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            param1=float(self.source_system_id),
            param2=float(self.source_component_id),
        )

    def publish_gimbal_command(self):
        # We exclusively send absolute positions. No velocities. No math. 
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            param1=float(self.commanded_pitch_deg),
            param2=float(self.commanded_yaw_deg),
            param3=float('nan'),
            param4=float('nan'),
            param5=0.0,
            param7=0.0,
        )

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = self.source_system_id
        msg.source_component = self.source_component_id
        msg.from_external = True
        msg.timestamp = self.timestamp_us()
        self.command_publisher.publish(msg)

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def update_commanded_orientation(self, dt):
        max_delta = self.command_slew_rate_deg_s * max(dt, 0.0)
        self.commanded_roll_deg = self.move_towards(
            self.commanded_roll_deg,
            self.target_roll_deg,
            max_delta,
        )
        self.commanded_pitch_deg = self.move_towards(
            self.commanded_pitch_deg,
            self.target_pitch_deg,
            max_delta,
        )
        self.commanded_yaw_deg = self.move_angle_towards(
            self.commanded_yaw_deg,
            self.target_yaw_deg,
            max_delta,
        )

    def move_towards(self, current_value, target_value, max_delta):
        delta = target_value - current_value
        if abs(delta) <= max_delta:
            return target_value
        if delta > 0.0:
            return current_value + max_delta
        return current_value - max_delta

    def move_angle_towards(self, current_value, target_value, max_delta):
        current_value = self.normalize_angle_deg(current_value)
        target_value = self.normalize_angle_deg(target_value)
        delta = self.normalize_angle_deg(target_value - current_value)
        if abs(delta) <= max_delta:
            return target_value
        if delta > 0.0:
            return self.normalize_angle_deg(current_value + max_delta)
        return self.normalize_angle_deg(current_value - max_delta)

    def normalize_angle_deg(self, angle_deg):
        normalized = float(angle_deg)
        while normalized > 180.0:
            normalized -= 360.0
        while normalized <= -180.0:
            normalized += 360.0
        return normalized


def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
