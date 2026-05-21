import json
import math

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


class BarrelDropMissionNode(Node):
    def __init__(self):
        super().__init__('barrel_drop_mission')

        self.declare_parameter('takeoff_height_m', 2.5)
        self.declare_parameter('cruise_height_m', 2.5)
        self.declare_parameter('barrel_drop_height_m', 1.55)
        self.declare_parameter('barrel_position_x_m', -30.0)
        self.declare_parameter('barrel_position_y_m', 0.0)
        self.declare_parameter('position_tolerance_m', 0.3)
        self.declare_parameter('altitude_tolerance_m', 0.22)
        self.declare_parameter('settle_time_s', 1.0)
        self.declare_parameter('release_retry_period_s', 1.0)
        self.declare_parameter('drop_offset_below_drone_m', 0.35)
        self.declare_parameter('hidden_ball_base_x_m', 1000.0)
        self.declare_parameter('hidden_ball_base_y_m', 1000.0)
        self.declare_parameter('hidden_ball_z_m', -10.0)
        self.declare_parameter('world_name', 'hydrolab_built')
        self.declare_parameter('drone_model_name', 'x500_depth_0')
        self.declare_parameter(
            'ball_model_names',
            ['ball_red', 'ball_blue', 'ball_yellow'],
        )
        self.declare_parameter('preferred_ball_model_name', 'ball_blue')

        self.takeoff_height_m = max(
            float(self.get_parameter('takeoff_height_m').value),
            0.5,
        )
        self.cruise_height_m = max(
            float(self.get_parameter('cruise_height_m').value),
            self.takeoff_height_m,
        )
        self.barrel_drop_height_m = max(
            float(self.get_parameter('barrel_drop_height_m').value),
            0.4,
        )
        self.barrel_position_x_m = float(
            self.get_parameter('barrel_position_x_m').value
        )
        self.barrel_position_y_m = float(
            self.get_parameter('barrel_position_y_m').value
        )
        self.position_tolerance_m = max(
            float(self.get_parameter('position_tolerance_m').value),
            0.05,
        )
        self.altitude_tolerance_m = max(
            float(self.get_parameter('altitude_tolerance_m').value),
            0.05,
        )
        self.settle_time_s = max(
            float(self.get_parameter('settle_time_s').value),
            0.1,
        )
        self.release_retry_period_s = max(
            float(self.get_parameter('release_retry_period_s').value),
            0.2,
        )
        self.drop_offset_below_drone_m = max(
            float(self.get_parameter('drop_offset_below_drone_m').value),
            0.05,
        )
        self.hidden_ball_base_x_m = float(
            self.get_parameter('hidden_ball_base_x_m').value
        )
        self.hidden_ball_base_y_m = float(
            self.get_parameter('hidden_ball_base_y_m').value
        )
        self.hidden_ball_z_m = float(
            self.get_parameter('hidden_ball_z_m').value
        )
        self.world_name = str(self.get_parameter('world_name').value).strip()
        self.drone_model_name = str(
            self.get_parameter('drone_model_name').value
        ).strip()
        self.ball_model_names = set(
            self.get_parameter('ball_model_names').value
        )
        self.preferred_ball_model_name = str(
            self.get_parameter('preferred_ball_model_name').value
        ).strip()

        self.target_publisher = self.create_publisher(
            Point,
            '/pilot/target_position',
            10,
        )

        self.home_position = None
        self.current_position = None
        self.drone_pose = None
        self.ball_poses = {}

        self.odom_subscriber = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.gz_node = GzNode()
        self.pose_topic = f'/world/{self.world_name}/pose/info'
        self.set_pose_service = f'/world/{self.world_name}/set_pose'
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
                f'Subscribed to Gazebo pose topic: {self.pose_topic}'
            )
        self.state = 'WAIT_FOR_ODOMETRY'
        self.state_since_ns = self.get_clock().now().nanoseconds
        self.last_release_attempt_ns = 0
        self.last_target = None
        self.release_done = False
        self.balls_hidden = False

        self.get_logger().info(
            'Barrel drop mission ready. The mission no longer waits for ball_grabber. '
            'It will fly to the barrel and spawn a ball below the drone for release.'
        )

    def odometry_callback(self, msg):
        self.current_position = [
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        ]

        if self.home_position is not None:
            return

        self.home_position = self.current_position.copy()
        self.get_logger().info(
            f'Home position locked at '
            f'({self.home_position[0]:.2f}, {self.home_position[1]:.2f}, {self.home_position[2]:.2f}).'
        )

    def pose_callback(self, msg):
        if len(msg.pose) > 0 and not hasattr(self, '_pose_log_once'):
            self.get_logger().info(
                f'Received pose/info with {len(msg.pose)} entities: '
                f'{[pose.name for pose in msg.pose[:10]]}'
            )
            self._pose_log_once = True

        for pose in msg.pose:
            tracked_name = self.resolve_tracked_entity_name(pose.name)
            if tracked_name is None:
                if 'ball' in pose.name.lower():
                    tracked_name = pose.name
                    self.get_logger().debug(f'Detected ball: {tracked_name}')
                else:
                    continue

            pose_data = {
                'entity_name': pose.name,
                'scoped_entity_name': pose.name,
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z),
            }

            if tracked_name == self.drone_model_name:
                self.drone_pose = pose_data
            elif tracked_name in self.ball_model_names or 'ball' in tracked_name.lower():
                self.ball_poses[tracked_name] = pose_data

    def timer_callback(self):
        if not self.balls_hidden and self.ball_poses:
            self.hide_visible_balls()

        if self.home_position is None or self.current_position is None:
            return

        if self.state == 'WAIT_FOR_ODOMETRY':
            self.transition_to('TAKEOFF')

        if self.state == 'TAKEOFF':
            target = self.make_target(
                self.home_position[0],
                self.home_position[1],
                self.altitude_to_ned(self.takeoff_height_m),
            )
            self.publish_target(target)
            if self.is_at_target(target):
                self.transition_to('GO_TO_BARREL')
            return

        if self.state == 'GO_TO_BARREL':
            target = self.make_target(
                self.barrel_position_x_m,
                self.barrel_position_y_m,
                self.altitude_to_ned(self.cruise_height_m),
            )
            self.publish_target(target)
            if self.is_at_target(target):
                self.transition_to('DESCEND_TO_DROP')
            return

        if self.state == 'DESCEND_TO_DROP':
            target = self.make_target(
                self.barrel_position_x_m,
                self.barrel_position_y_m,
                self.altitude_to_ned(self.barrel_drop_height_m),
            )
            self.publish_target(target)
            if self.is_at_target(target):
                self.transition_to('SETTLE_OVER_BARREL')
            return

        if self.state == 'SETTLE_OVER_BARREL':
            target = self.make_target(
                self.barrel_position_x_m,
                self.barrel_position_y_m,
                self.altitude_to_ned(self.barrel_drop_height_m),
            )
            self.publish_target(target)
            if self.seconds_in_state() >= self.settle_time_s:
                self.transition_to('RELEASE_PAYLOAD')
            return

        if self.state == 'RELEASE_PAYLOAD':
            target = self.make_target(
                self.barrel_position_x_m,
                self.barrel_position_y_m,
                self.altitude_to_ned(self.barrel_drop_height_m),
            )
            self.publish_target(target)

            now_ns = self.get_clock().now().nanoseconds
            if (
                self.last_release_attempt_ns == 0
                or now_ns - self.last_release_attempt_ns
                >= int(self.release_retry_period_s * 1e9)
            ):
                self.publish_release_command()
                self.last_release_attempt_ns = now_ns

            if self.release_done:
                self.transition_to('MISSION_COMPLETE')
            return

        if self.state == 'MISSION_COMPLETE':
            target = self.make_target(
                self.barrel_position_x_m,
                self.barrel_position_y_m,
                self.altitude_to_ned(self.barrel_drop_height_m),
            )
            self.publish_target(target)

    def transition_to(self, new_state):
        if new_state == self.state:
            return

        self.state = new_state
        self.state_since_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Mission state -> {new_state}')

    def seconds_in_state(self):
        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self.state_since_ns) / 1e9

    def altitude_to_ned(self, altitude_m):
        return self.home_position[2] - altitude_m

    def make_target(self, x, y, z):
        return [float(x), float(y), float(z)]

    def publish_target(self, target):
        if self.last_target is not None:
            deltas = [
                abs(target[index] - self.last_target[index])
                for index in range(3)
            ]
            if max(deltas) < 1e-4:
                return

        msg = Point()
        msg.x = target[0]
        msg.y = target[1]
        msg.z = target[2]
        self.target_publisher.publish(msg)
        self.last_target = target.copy()

    def publish_release_command(self):
        if self.release_done:
            return

        if self.spawn_ball_below_drone():
            self.release_done = True
        else:
            self.get_logger().warn(
                'Release requested, but no ball was available to spawn.'
            )

    def spawn_ball_below_drone(self):
        drone_position = None

        if self.drone_pose is not None:
            drone_position = self.drone_pose
            self.get_logger().debug('Using Gazebo drone_pose for release.')
        elif self.current_position is not None:
            drone_position = {
                'x': float(self.current_position[0]),
                'y': float(self.current_position[1]),
                'z': -float(self.current_position[2]),
            }
            self.get_logger().info(
                f'Using odometry as fallback (with NED->Gazebo conversion): '
                f'({drone_position["x"]:.2f}, {drone_position["y"]:.2f}, {drone_position["z"]:.2f})'
            )
        else:
            self.get_logger().error('Drone pose and odometry are unavailable for release.')
            return False

        self.get_logger().info(
            f'Current ball_poses available: {list(self.ball_poses.keys())}'
        )

        candidate_name = self.preferred_ball_model_name
        if candidate_name not in self.ball_poses:
            self.get_logger().info(
                f'Preferred ball "{candidate_name}" not found in ball_poses. Searching alternatives...'
            )
            candidate_name = None
            for name in sorted(self.ball_poses.keys()):
                if name in self.ball_model_names or 'ball' in name.lower():
                    candidate_name = name
                    self.get_logger().info(f'Found ball candidate in ball_poses: {candidate_name}')
                    break

        # FALLBACK: If no ball found in pose_callback, try direct names
        if candidate_name is None:
            self.get_logger().warn(
                'No balls found in ball_poses. Attempting fallback with hardcoded ball names.'
            )
            for fallback_name in ['ball_blue', 'ball_red', 'ball_yellow']:
                fallback_pose = {
                    'entity_name': fallback_name,
                    'scoped_entity_name': fallback_name,
                    'x': 5.0,
                    'y': 0.0,
                    'z': 0.09,
                }
                self.get_logger().info(f'Attempting fallback with {fallback_name}')
                if self.try_set_and_release(fallback_name, fallback_pose, drone_position):
                    return True

        if candidate_name is None:
            self.get_logger().error(
                'No visible ball model found to spawn. Available ball candidates: '
                f'{sorted(self.ball_poses.keys())}'
            )
            return False

        ball_pose = self.ball_poses[candidate_name].copy()
        self.get_logger().info(
            f'Selected ball "{candidate_name}" at ({ball_pose["x"]:.2f}, {ball_pose["y"]:.2f}, {ball_pose["z"]:.2f})'
        )

        return self.try_set_and_release(candidate_name, ball_pose, drone_position)

    def hide_visible_balls(self):
        hidden_any_ball = False
        for index, ball_name in enumerate(sorted(self.ball_poses.keys())):
            ball_pose = self.ball_poses[ball_name].copy()
            hidden_pose = {
                'x': self.hidden_ball_base_x_m + index * 2.0,
                'y': self.hidden_ball_base_y_m,
                'z': self.hidden_ball_z_m,
            }
            if not self.set_ball_pose(ball_pose, hidden_pose):
                self.get_logger().warn(
                    f'Failed to hide ball "{ball_name}" before the mission start.'
                )
                continue

            self.ball_poses[ball_name].update(hidden_pose)
            hidden_any_ball = True

        if hidden_any_ball:
            self.balls_hidden = True
            self.get_logger().info(
                'Hidden visible balls outside the scene. The selected ball will '
                'appear only at the drop point.'
            )

    def try_set_and_release(self, ball_name, ball_pose, drone_position):
        release_pose = {
            'x': float(drone_position['x']) + 0.0,
            'y': float(drone_position['y']) + 0.0,
            'z': max(
                float(drone_position['z']) - self.drop_offset_below_drone_m,
                0.05,
            ),
        }
        self.get_logger().info(
            f'Target release pose in Gazebo: '
            f'({release_pose["x"]:.2f}, {release_pose["y"]:.2f}, {release_pose["z"]:.2f}) m '
            f'with a {self.drop_offset_below_drone_m:.2f} m downward offset from the drone'
        )

        if not self.set_ball_pose(ball_pose, release_pose):
            self.get_logger().error('set_ball_pose failed.')
            return False

        if ball_name in self.ball_poses:
            self.ball_poses[ball_name].update(release_pose)
        self.get_logger().info(
            f'✅ Ball "{ball_name}" spawned at ({release_pose["x"]:.2f}, {release_pose["y"]:.2f}, {release_pose["z"]:.2f}) - should now be falling.'
        )
        return True

    def set_ball_pose(self, ball_reference, target_position):
        candidate_names = []
        for key in ('entity_name', 'scoped_entity_name'):
            name = str(ball_reference.get(key, '')).strip()
            if name and name not in candidate_names:
                candidate_names.append(name)

        self.get_logger().info(
            f'Attempting to set ball pose to ({target_position["x"]:.2f}, '
            f'{target_position["y"]:.2f}, {target_position["z"]:.2f}). '
            f'Candidate names: {candidate_names}'
        )

        for entity_name in candidate_names:
            target_pose = Pose()
            target_pose.name = entity_name
            target_pose.position.x = float(target_position['x'])
            target_pose.position.y = float(target_position['y'])
            target_pose.position.z = float(target_position['z'])
            target_pose.orientation.x = 0.0
            target_pose.orientation.y = 0.0
            target_pose.orientation.z = 0.0
            target_pose.orientation.w = 1.0

            self.get_logger().debug(
                f'Calling set_pose for entity "{entity_name}" at '
                f'({target_position["x"]:.2f}, {target_position["y"]:.2f}, {target_position["z"]:.2f})'
            )

            success, response = self.gz_node.request(
                self.set_pose_service,
                target_pose,
                Pose,
                Boolean,
                1000,
            )
            self.get_logger().info(
                f'set_pose result: success={success}, response.data={response.data if success else "N/A"}'
            )
            if success and response.data:
                self.get_logger().info(f'Successfully moved ball to new position.')
                return True
            else:
                self.get_logger().warn(
                    f'Failed to set pose for "{entity_name}". '
                    f'success={success}, response={response.data if success else "error"}'
                )

        self.get_logger().error('set_ball_pose failed for all candidate names.')
        return False

    def resolve_tracked_entity_name(self, entity_name):
        parts = [part for part in str(entity_name).split('::') if part]

        if self.drone_model_name in parts:
            return self.drone_model_name

        for ball_name in self.ball_model_names:
            if ball_name in parts:
                return ball_name

        return None

    def is_at_target(self, target):
        horizontal_error = math.hypot(
            self.current_position[0] - target[0],
            self.current_position[1] - target[1],
        )
        vertical_error = abs(self.current_position[2] - target[2])
        return (
            horizontal_error <= self.position_tolerance_m
            and vertical_error <= self.altitude_tolerance_m
        )


def main(args=None):
    rclpy.init(args=args)
    node = BarrelDropMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
