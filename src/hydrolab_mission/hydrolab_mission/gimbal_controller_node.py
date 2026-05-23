import math

from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, VehicleOdometry
import rclpy
from rclpy.node import Node


class GimbalControllerNode(Node):
    def __init__(self):
        super().__init__('gimbal_controller')

        self.declare_parameter('source_system_id', 1)
        self.declare_parameter('source_component_id', 191)
        self.declare_parameter('nadir_pitch_deg', -90.0)
        self.declare_parameter('nadir_earth_yaw_deg', 0.0)
        self.declare_parameter('odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('stabilization_enabled_search', True)
        self.declare_parameter('stabilization_enabled_lock', True)
        self.declare_parameter('stabilization_pitch_gain', 1.0)
        self.declare_parameter('stabilization_roll_to_yaw_gain', 0.25)
        self.declare_parameter('stabilization_max_pitch_offset_deg', 15.0)
        self.declare_parameter('stabilization_max_yaw_offset_deg', 10.0)
        self.declare_parameter('stabilization_attitude_lpf_alpha', 0.2)
        
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
        self.nadir_pitch_deg = float(self.get_parameter('nadir_pitch_deg').value)
        self.nadir_earth_yaw_deg = float(
            self.get_parameter('nadir_earth_yaw_deg').value
        )
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.stabilization_enabled_search = bool(
            self.get_parameter('stabilization_enabled_search').value
        )
        self.stabilization_enabled_lock = bool(
            self.get_parameter('stabilization_enabled_lock').value
        )
        self.stabilization_pitch_gain = float(
            self.get_parameter('stabilization_pitch_gain').value
        )
        self.stabilization_roll_to_yaw_gain = float(
            self.get_parameter('stabilization_roll_to_yaw_gain').value
        )
        self.stabilization_max_pitch_offset_deg = max(
            0.0,
            float(self.get_parameter('stabilization_max_pitch_offset_deg').value),
        )
        self.stabilization_max_yaw_offset_deg = max(
            0.0,
            float(self.get_parameter('stabilization_max_yaw_offset_deg').value),
        )
        self.stabilization_attitude_lpf_alpha = float(
            self.get_parameter('stabilization_attitude_lpf_alpha').value
        )
        self.stabilization_attitude_lpf_alpha = max(
            0.0,
            min(1.0, self.stabilization_attitude_lpf_alpha),
        )
        # Publish commands at 10Hz so PX4 doesn't time out.
        self.command_interval_s = 0.1
        
        wp_list = self.get_parameter('search_waypoints').value
        # convert flat list into list of tuples (pitch, yaw)
        self.search_waypoints = [(wp_list[i], wp_list[i+1]) for i in range(0, len(wp_list), 2)]


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
        self.state = "IDLE"  # IDLE, SEARCHING, LOCKED, NADIR_EARTH
        self.last_target_time_s = 0.0
        
        self.status_publisher = self.create_publisher(String, '/gimbal/status', 10)
        self.attitude_publisher = self.create_publisher(Vector3, '/gimbal/current_orientation_deg', 10)
        
        self.command_subscription = self.create_subscription(
            String,
            '/gimbal/command',
            self.command_callback,
            10,
        )
        self.odom_subscription = self.create_subscription(
            VehicleOdometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        
        # Base orientation before stabilization feed-forward is applied.
        self.base_roll_deg = 0.0
        self.base_pitch_deg = 0.0
        self.base_yaw_deg = 0.0
        # Final command orientation after stabilization compensation.
        self.target_roll_deg = 0.0
        self.target_pitch_deg = 0.0
        self.target_yaw_deg = 0.0
        self.filtered_vehicle_roll_deg = 0.0
        self.filtered_vehicle_pitch_deg = 0.0
        self.has_vehicle_attitude = False
        self.has_target = False

        # ==========================================
        # THE DISCRETE SEQUENCE (Pitch, Yaw)
        # ==========================================
        self.current_waypoint_index = 0
        self.time_at_waypoint_s = 0.0
        self.waypoint_duration_s = 2.5 # Wait 2.5 seconds at each point to let the gimbal settle

        
        self.last_timer_time = self.get_clock().now()

        self.timer = self.create_timer(
            self.command_interval_s,
            self.timer_callback,
        )
        self.get_logger().info('Gimbal Controller initialized with Discrete Sequence.')

    def target_callback(self, msg):
        if self.state == "NADIR_EARTH":
            return
        self.base_roll_deg = float(msg.x)
        self.base_pitch_deg = float(msg.y)
        self.base_yaw_deg = float(msg.z)
        self.state = "LOCKED"
        self.last_target_time_s = self.get_clock().now().nanoseconds / 1e9
        self.has_target = True

    def timer_callback(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        dt = (now - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = now
        
        # If locked but we haven't seen the target in 2 seconds, go back to searching
        if self.state == "LOCKED" and (now_s - self.last_target_time_s) > 2.0:
            self.state = "SEARCHING"
            self.get_logger().info("Target lost. Resuming Discrete Sequence Search.")
            self.time_at_waypoint_s = 0.0

        if self.state == "SEARCHING":
            self.time_at_waypoint_s += dt
            
            # If we've been at this waypoint long enough, move to the next one
            if self.time_at_waypoint_s >= self.waypoint_duration_s:
                self.time_at_waypoint_s = 0.0
                self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.search_waypoints)
                self.get_logger().info(f"Gimbal Sequence: Moving to step {self.current_waypoint_index + 1}/{len(self.search_waypoints)}")

            # Grab the coordinates for the current step in the sequence
            target_p, target_y = self.search_waypoints[self.current_waypoint_index]
            self.base_roll_deg = 0.0
            self.base_pitch_deg = target_p
            self.base_yaw_deg = target_y
            self.has_target = True
        elif self.state == "NADIR_EARTH":
            # Keep gimbal at a nadir orientation while hover controller runs.
            self.base_roll_deg = 0.0
            self.base_pitch_deg = self.nadir_pitch_deg
            self.base_yaw_deg = self.nadir_earth_yaw_deg
            self.has_target = True

        if not self.has_target:
            return
        
        self.update_command_orientation()

        if not hasattr(self, 'gimbal_configured'):
            self.configure_gimbal_manager()
            self.gimbal_configured = True

        self.publish_gimbal_command()
        
        msg = String()
        msg.data = f"STATUS_{self.state}"
        self.status_publisher.publish(msg)

        att_msg = Vector3()
        att_msg.x = float(self.target_roll_deg)
        att_msg.y = float(self.target_pitch_deg)
        att_msg.z = float(self.target_yaw_deg)
        self.attitude_publisher.publish(att_msg)

    def command_callback(self, msg):
        cmd = msg.data.upper()
        if cmd == "COMMAND_SEARCH":
            self.state = "SEARCHING"
            self.current_waypoint_index = 0
            self.time_at_waypoint_s = 0.0
            
            self.get_logger().info("Gimbal received command: Entering Discrete Sequence Search.")
        elif cmd == "COMMAND_EARTH_NADIR":
            self.state = "NADIR_EARTH"
            self.base_roll_deg = 0.0
            self.base_pitch_deg = self.nadir_pitch_deg
            self.base_yaw_deg = self.nadir_earth_yaw_deg
            self.update_command_orientation()
            self.has_target = True
            self.get_logger().info(
                "Gimbal received command: Holding earth-referenced nadir "
                f"(pitch={self.nadir_pitch_deg:.1f}, yaw={self.nadir_earth_yaw_deg:.1f})."
            )
        elif cmd == "COMMAND_RESET":
            self.state = "IDLE"
            self.base_roll_deg = 0.0
            self.base_pitch_deg = 0.0
            self.base_yaw_deg = 0.0
            self.update_command_orientation()
            self.has_target = True
            self.get_logger().info("Gimbal received command: Resetting to IDLE neutral.")

    def odom_callback(self, msg):
        roll_deg, pitch_deg = self.quaternion_to_roll_pitch_deg(msg.q)
        if not self.has_vehicle_attitude:
            self.filtered_vehicle_roll_deg = roll_deg
            self.filtered_vehicle_pitch_deg = pitch_deg
            self.has_vehicle_attitude = True
            return

        alpha = self.stabilization_attitude_lpf_alpha
        self.filtered_vehicle_roll_deg = (
            alpha * roll_deg
        ) + ((1.0 - alpha) * self.filtered_vehicle_roll_deg)
        self.filtered_vehicle_pitch_deg = (
            alpha * pitch_deg
        ) + ((1.0 - alpha) * self.filtered_vehicle_pitch_deg)

    def update_command_orientation(self):
        self.target_roll_deg = self.base_roll_deg
        self.target_pitch_deg = self.base_pitch_deg
        self.target_yaw_deg = self.base_yaw_deg

        if not self.should_apply_stabilization():
            return

        pitch_offset_deg = -self.stabilization_pitch_gain * self.filtered_vehicle_pitch_deg
        yaw_offset_deg = -self.stabilization_roll_to_yaw_gain * self.filtered_vehicle_roll_deg
        pitch_offset_deg = max(
            -self.stabilization_max_pitch_offset_deg,
            min(self.stabilization_max_pitch_offset_deg, pitch_offset_deg),
        )
        yaw_offset_deg = max(
            -self.stabilization_max_yaw_offset_deg,
            min(self.stabilization_max_yaw_offset_deg, yaw_offset_deg),
        )
        self.target_pitch_deg += pitch_offset_deg
        self.target_yaw_deg += yaw_offset_deg

    def should_apply_stabilization(self):
        if not self.has_vehicle_attitude:
            return False
        if self.state == "SEARCHING":
            return self.stabilization_enabled_search
        if self.state == "LOCKED":
            return self.stabilization_enabled_lock
        return False

    @staticmethod
    def quaternion_to_roll_pitch_deg(q):
        qw = float(q[0])
        qx = float(q[1])
        qy = float(q[2])
        qz = float(q[3])

        sinr_cosp = 2.0 * ((qw * qx) + (qy * qz))
        cosr_cosp = 1.0 - 2.0 * ((qx * qx) + (qy * qy))
        roll_rad = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * ((qw * qy) - (qz * qx))
        sinp = max(-1.0, min(1.0, sinp))
        pitch_rad = math.asin(sinp)

        return math.degrees(roll_rad), math.degrees(pitch_rad)

    def configure_gimbal_manager(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            param1=float(self.source_system_id),
            param2=float(self.source_component_id),
        )

    def publish_gimbal_command(self):
        # We exclusively send absolute positions. No velocities. No math. 
        flags = 16.0 if self.state == "NADIR_EARTH" else 0.0
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            param1=float(self.target_pitch_deg),
            param2=float(self.target_yaw_deg),
            param3=float('nan'),
            param4=float('nan'),
            param5=flags,
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


def main(args=None):
    rclpy.init(args=args)
    node = GimbalControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
