import json
import math
from threading import Lock
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class BallGrabberNode(Node):
    def __init__(self):
        super().__init__('ball_grabber')

        self.declare_parameter('world_name', 'last_mile_logistics_randomized')
        self.declare_parameter('drone_model_name', 'x500_gimbal_0')
        self.declare_parameter(
            'ball_model_names',
            ['ball_red', 'ball_blue', 'ball_yellow'],
        )
        self.declare_parameter('attach_xy_threshold_m', 0.35)
        self.declare_parameter('attach_height_threshold_m', 3.0)
        self.declare_parameter('visual_attach_xy_threshold_m', 0.9)
        self.declare_parameter('visual_center_tolerance_u', 0.08)
        self.declare_parameter('visual_center_tolerance_v', 0.15)
        self.declare_parameter('visual_detection_max_age_s', 0.8)
        self.declare_parameter('carry_offset_x_m', 0.0)
        self.declare_parameter('carry_offset_y_m', 0.0)
        self.declare_parameter('carry_offset_z_m', -0.18)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('service_timeout_ms', 1000)
        self.declare_parameter('enabled_at_start', False)
        self.declare_parameter('hidden_ball_base_x_m', 1000.0)
        self.declare_parameter('hidden_ball_base_y_m', 1000.0)
        self.declare_parameter('hidden_ball_z_m', -10.0)
        self.declare_parameter('collection_action', 'remove')
        self.declare_parameter('preload_ball_model_name', '')

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
        self.visual_attach_xy_threshold_m = max(
            float(self.get_parameter('visual_attach_xy_threshold_m').value),
            self.attach_xy_threshold_m,
        )
        self.visual_center_tolerance_u = max(
            float(self.get_parameter('visual_center_tolerance_u').value),
            0.01,
        )
        self.visual_center_tolerance_v = max(
            float(self.get_parameter('visual_center_tolerance_v').value),
            0.01,
        )
        self.visual_detection_max_age_s = max(
            float(self.get_parameter('visual_detection_max_age_s').value),
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
        self.hidden_ball_base_x_m = float(
            self.get_parameter('hidden_ball_base_x_m').value
        )
        self.hidden_ball_base_y_m = float(
            self.get_parameter('hidden_ball_base_y_m').value
        )
        self.hidden_ball_z_m = float(
            self.get_parameter('hidden_ball_z_m').value
        )
        self.collection_action = str(
            self.get_parameter('collection_action').value
        ).strip().lower()
        if self.collection_action not in ('hide', 'remove'):
            self.get_logger().warn(
                f'Unsupported collection_action="{self.collection_action}". '
                'Falling back to "remove".'
            )
            self.collection_action = 'remove'
        self.preload_ball_model_name = str(
            self.get_parameter('preload_ball_model_name').value
        ).strip()

        self.pose_topic = f'/world/{self.world_name}/pose/info'
        self.remove_entity_service = f'/world/{self.world_name}/remove'
        self.set_pose_service = f'/world/{self.world_name}/set_pose'
        self.gz_node = GzNode()

        self.state_lock = Lock()
        self.drone_pose = None
        self.ball_poses = {}
        self.target_color = 'all'
        self.latest_detection = {'active': False, 'detections': []}
        self.latest_detection_ns = 0
        self.collected_ball_names = set()
        self.collected_ball_records = []
        self.last_service_warning_ns = 0
        self.last_wait_log_ns = 0
        self.logged_pose_sources = set()
        self.preload_attempted = False

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE

        self.enable_subscription = self.create_subscription(
            Bool,
            '/ball_grabber/enabled',
            self.enable_callback,
            10,
        )
        self.target_color_subscription = self.create_subscription(
            String,
            '/ball_grabber/target_color',
            self.target_color_callback,
            10,
        )
        self.release_subscription = self.create_subscription(
            Bool,
            '/ball_grabber/release',
            self.release_callback,
            10,
        )
        self.detection_subscription = self.create_subscription(
            String,
            '/vision/ball_detection/result',
            self.detection_callback,
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            '/ball_grabber/state',
            state_qos,
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

        self.publish_state()

    def enable_callback(self, msg):
        requested_state = bool(msg.data)
        if requested_state == self.acquire_enabled:
            return

        self.acquire_enabled = requested_state
        state_label = 'enabled' if self.acquire_enabled else 'disabled'
        self.get_logger().info(f'Ball grabber acquisition {state_label}.')
        self.publish_state()

    def target_color_callback(self, msg):
        requested_color = msg.data.strip().lower()
        if requested_color in ('', 'all', 'any', 'none'):
            requested_color = 'all'
        else:
            requested_color = requested_color

        if requested_color == self.target_color:
            return

        self.target_color = requested_color

        self.get_logger().info(
            f'Ball grabber target color set to: {self.target_color}'
        )
        self.publish_state()

    def detection_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        with self.state_lock:
            self.latest_detection = payload
            self.latest_detection_ns = time.time_ns()

    def release_callback(self, msg):
        if not bool(msg.data):
            return

        self.release_latest_ball()

    def pose_callback(self, msg):
        with self.state_lock:
            for pose in msg.pose:
                tracked_name = self.resolve_tracked_entity_name(pose.name)
                if tracked_name is None:
                    continue

                pose_data = {
                    'entity_name': tracked_name,
                    'scoped_entity_name': pose.name,
                    'entity_id': int(pose.id),
                    'x': float(pose.position.x),
                    'y': float(pose.position.y),
                    'z': float(pose.position.z),
                    'qx': float(pose.orientation.x),
                    'qy': float(pose.orientation.y),
                    'qz': float(pose.orientation.z),
                    'qw': float(pose.orientation.w),
                }

                if tracked_name == self.drone_model_name:
                    self.drone_pose = pose_data
                elif tracked_name in self.ball_model_names:
                    if tracked_name in self.collected_ball_names:
                        self.ball_poses.pop(tracked_name, None)
                        continue
                    self.ball_poses[tracked_name] = pose_data

                    log_key = (tracked_name, pose.name)
                    if log_key not in self.logged_pose_sources:
                        self.get_logger().info(
                            f'Tracking Gazebo pose "{pose.name}" as model "{tracked_name}".'
                        )
                        self.logged_pose_sources.add(log_key)

    def timer_callback(self):
        if not self.preload_attempted and self.preload_ball_model_name:
            self.try_preload_ball()

        if not self.acquire_enabled:
            return

        self.try_collect_ball()

    def try_preload_ball(self):
        with self.state_lock:
            ball_name = self.preload_ball_model_name
            if not ball_name:
                available_ball_names = sorted(self.ball_poses.keys())
                if not available_ball_names:
                    return
                ball_name = available_ball_names[0]

            if any(
                record.get('model_name') == ball_name
                for record in self.collected_ball_records
            ):
                self.preload_attempted = True
                return

            ball_pose = self.ball_poses.get(ball_name)
            if ball_pose is None:
                available_ball_names = sorted(self.ball_poses.keys())
                if not available_ball_names:
                    return
                fallback_ball_name = available_ball_names[0]
                ball_pose = self.ball_poses.get(fallback_ball_name)
                if ball_pose is None:
                    return
                if fallback_ball_name != ball_name:
                    self.get_logger().warn(
                        f'Preferred preload ball "{ball_name}" is unavailable. '
                        f'Falling back to "{fallback_ball_name}".'
                    )
                ball_name = fallback_ball_name

        if not self.hide_ball_from_world(ball_pose):
            return

        ball_color = self.color_from_ball_name(ball_name)
        with self.state_lock:
            self.collected_ball_names.add(ball_name)
            self.ball_poses.pop(ball_name, None)
            self.collected_ball_records.append(
                {
                    'color': ball_color,
                    'model_name': ball_name,
                    'entity_name': ball_pose['entity_name'],
                    'scoped_entity_name': ball_pose['scoped_entity_name'],
                    'storage_mode': 'hidden',
                }
            )
            self.preload_attempted = True

        self.get_logger().info(
            f'Preloaded payload "{ball_name}" for release-only mission.'
        )
        self.publish_state()

    def try_collect_ball(self):
        with self.state_lock:
            drone_pose = None if self.drone_pose is None else self.drone_pose.copy()
            ball_poses = {
                key: value.copy()
                for key, value in self.ball_poses.items()
            }
            detection_payload = dict(self.latest_detection)
            detection_ns = int(self.latest_detection_ns)

        if drone_pose is None:
            return

        best_ball_name = None
        best_ball_pose = None
        best_distance = None
        best_vertical_distance = None
        best_match_mode = 'strict'
        nearest_ball_name = None
        nearest_horizontal_distance = None
        nearest_vertical_distance = None
        visual_lock = self.has_visual_pickup_lock(detection_payload, detection_ns)

        for ball_name, ball_pose in ball_poses.items():
            if not self.ball_matches_target_color(ball_name):
                continue

            horizontal_distance = math.hypot(
                drone_pose['x'] - ball_pose['x'],
                drone_pose['y'] - ball_pose['y'],
            )
            # Gazebo pose/info is in the simulator world frame (z-up), so when the
            # drone hovers above the ball its z is greater than the ball z.
            vertical_distance = drone_pose['z'] - ball_pose['z']

            if (
                nearest_horizontal_distance is None
                or horizontal_distance < nearest_horizontal_distance
            ):
                nearest_ball_name = ball_name
                nearest_horizontal_distance = horizontal_distance
                nearest_vertical_distance = vertical_distance

            if vertical_distance < 0.0:
                continue
            if vertical_distance > self.attach_height_threshold_m:
                continue

            strict_match = horizontal_distance <= self.attach_xy_threshold_m
            visual_match = (
                visual_lock
                and horizontal_distance <= self.visual_attach_xy_threshold_m
            )
            visual_override_match = visual_lock

            if not strict_match and not visual_match and not visual_override_match:
                continue

            if best_distance is None or horizontal_distance < best_distance:
                best_distance = horizontal_distance
                best_vertical_distance = vertical_distance
                best_ball_name = ball_name
                best_ball_pose = ball_pose
                if strict_match:
                    best_match_mode = 'strict'
                elif visual_match:
                    best_match_mode = 'visual_xy'
                else:
                    best_match_mode = 'visual_override'

        if best_ball_name is None or best_ball_pose is None:
            self.log_waiting_for_pickup(
                visual_lock=visual_lock,
                nearest_ball_name=nearest_ball_name,
                nearest_horizontal_distance=nearest_horizontal_distance,
                nearest_vertical_distance=nearest_vertical_distance,
            )
            return

        storage_mode = self.collection_action
        if storage_mode == 'remove':
            removed_from_world = self.remove_ball_from_world(best_ball_pose)
            if removed_from_world:
                storage_mode = 'removed'
            else:
                if not self.hide_ball_from_world(best_ball_pose):
                    return
                storage_mode = 'hidden'
        else:
            if not self.hide_ball_from_world(best_ball_pose):
                return
            storage_mode = 'hidden'

        ball_color = self.color_from_ball_name(best_ball_name)
        with self.state_lock:
            self.collected_ball_names.add(best_ball_name)
            self.ball_poses.pop(best_ball_name, None)
            self.collected_ball_records.append(
                {
                    'color': ball_color,
                    'model_name': best_ball_name,
                    'entity_name': best_ball_pose['entity_name'],
                    'scoped_entity_name': best_ball_pose['scoped_entity_name'],
                    'storage_mode': storage_mode,
                }
            )

        self.acquire_enabled = False
        self.get_logger().info(
            f'Collected {ball_color} ball "{best_ball_name}" and stored it as {storage_mode}. '
            f'xy={best_distance:.2f} m, dz={best_vertical_distance:.2f} m, '
            f'visual_lock={visual_lock}, match_mode={best_match_mode}, '
            f'collection_action={self.collection_action}'
        )
        self.publish_state()

    def release_latest_ball(self):
        with self.state_lock:
            drone_pose = None if self.drone_pose is None else self.drone_pose.copy()
            ball_record = (
                None
                if not self.collected_ball_records
                else self.collected_ball_records[-1].copy()
            )

        if drone_pose is None:
            self.get_logger().warn('Release requested before the drone pose is available.')
            return

        if ball_record is None:
            spawned = self.spawn_available_ball_below_drone(drone_pose)
            if not spawned:
                self.get_logger().warn(
                    'Release requested, but there is no hidden payload and no visible ball available to drop.'
                )
            return

        if ball_record.get('storage_mode') != 'hidden':
            self.get_logger().warn(
                'Release requested for a payload collected with removal mode. '
                'Use collection_action=hide to make payloads releasable.'
            )
            return

        release_pose = {
            'x': float(drone_pose['x']) + self.carry_offset_x_m,
            'y': float(drone_pose['y']) + self.carry_offset_y_m,
            'z': max(
                float(drone_pose['z']) + self.carry_offset_z_m,
                0.05,
            ),
        }
        released = self.set_ball_pose(ball_record, release_pose)
        if not released:
            self.get_logger().warn(
                f'Failed to release payload "{ball_record.get("model_name", "unknown")}".'
            )
            return

        with self.state_lock:
            model_name = str(ball_record.get('model_name', '')).strip()
            if model_name:
                self.collected_ball_names.discard(model_name)
            if self.collected_ball_records:
                self.collected_ball_records.pop()
            if model_name:
                self.ball_poses[model_name] = {
                    'entity_name': str(ball_record.get('entity_name', model_name)),
                    'scoped_entity_name': str(
                        ball_record.get('scoped_entity_name', model_name)
                    ),
                    'entity_id': -1,
                    'x': release_pose['x'],
                    'y': release_pose['y'],
                    'z': release_pose['z'],
                    'qx': 0.0,
                    'qy': 0.0,
                    'qz': 0.0,
                    'qw': 1.0,
                }

        self.get_logger().info(
            f'Released payload "{ball_record.get("model_name", "unknown")}" at '
            f'({release_pose["x"]:.2f}, {release_pose["y"]:.2f}, {release_pose["z"]:.2f}).'
        )
        self.publish_state()

    def spawn_available_ball_below_drone(self, drone_pose):
        with self.state_lock:
            preferred_ball_name = self.preload_ball_model_name
            candidate_ball_name = ''
            candidate_ball_pose = None

            if preferred_ball_name:
                candidate_ball_pose = self.ball_poses.get(preferred_ball_name)
                if candidate_ball_pose is not None:
                    candidate_ball_name = preferred_ball_name

            if candidate_ball_pose is None:
                available_ball_names = sorted(self.ball_poses.keys())
                if not available_ball_names:
                    return False
                candidate_ball_name = available_ball_names[0]
                candidate_ball_pose = self.ball_poses.get(candidate_ball_name)
                if candidate_ball_pose is None:
                    return False
                candidate_ball_pose = candidate_ball_pose.copy()
            else:
                candidate_ball_pose = candidate_ball_pose.copy()

        release_pose = {
            'x': float(drone_pose['x']) + self.carry_offset_x_m,
            'y': float(drone_pose['y']) + self.carry_offset_y_m,
            'z': max(
                float(drone_pose['z']) + self.carry_offset_z_m,
                0.05,
            ),
        }
        released = self.set_ball_pose(candidate_ball_pose, release_pose)
        if not released:
            return False

        self.get_logger().info(
            f'Spawned visible ball "{candidate_ball_name}" below the drone at '
            f'({release_pose["x"]:.2f}, {release_pose["y"]:.2f}, {release_pose["z"]:.2f}).'
        )
        return True

    def remove_ball_from_world(self, ball_pose):
        entity = Entity()
        entity.name = str(ball_pose['entity_name'])
        entity.type = Entity.MODEL

        success, response = self.gz_node.request(
            self.remove_entity_service,
            entity,
            Entity,
            Boolean,
            self.service_timeout_ms,
        )
        if success and response.data:
            return True

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_service_warning_ns >= int(2e9):
            self.get_logger().warn(
                'Failed to remove collected ball via Gazebo remove service. '
                f'Service: {self.remove_entity_service}. Falling back to set_pose.'
            )
            self.last_service_warning_ns = now_ns
        return False

    def hide_ball_from_world(self, ball_pose):
        hide_slot = len(self.collected_ball_records)
        target_pose = {
            'x': self.hidden_ball_base_x_m + hide_slot * 2.0,
            'y': self.hidden_ball_base_y_m,
            'z': self.hidden_ball_z_m,
        }
        if self.set_ball_pose(ball_pose, target_pose):
            return True

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_service_warning_ns >= int(2e9):
            self.get_logger().warn(
                'Failed to hide collected ball in Gazebo. '
                f'Service: {self.set_pose_service}'
            )
            self.last_service_warning_ns = now_ns
        return False

    def set_ball_pose(self, ball_reference, target_position):
        candidate_names = []
        for key in ('entity_name', 'scoped_entity_name'):
            name = str(ball_reference.get(key, '')).strip()
            if name and name not in candidate_names:
                candidate_names.append(name)

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

            success, response = self.gz_node.request(
                self.set_pose_service,
                target_pose,
                Pose,
                Boolean,
                self.service_timeout_ms,
            )
            if success and response.data:
                return True

        return False

    def ball_matches_target_color(self, ball_name):
        if self.target_color == 'all':
            return True
        return self.color_from_ball_name(ball_name) == self.target_color

    def color_from_ball_name(self, ball_name):
        short_name = self.short_name(ball_name).lower()
        for color in ('red', 'blue', 'yellow'):
            if short_name.endswith(color):
                return color
        return short_name.replace('ball_', '', 1)

    def resolve_tracked_entity_name(self, entity_name):
        parts = [part for part in str(entity_name).split('::') if part]

        if self.drone_model_name in parts:
            return self.drone_model_name

        for ball_name in self.ball_model_names:
            if ball_name in parts:
                return ball_name

        return None

    def has_visual_pickup_lock(self, detection_payload, detection_ns):
        if not detection_payload.get('active', False):
            return False
        if self.target_color == 'all':
            return False
        if detection_ns == 0:
            return False
        if time.time_ns() - detection_ns > int(self.visual_detection_max_age_s * 1e9):
            return False

        for detection in detection_payload.get('detections', []):
            if detection.get('color', '').lower() != self.target_color:
                continue

            center_uv = detection.get('center_uv', [0.5, 0.5])
            if len(center_uv) != 2:
                continue

            error_u = abs(float(center_uv[0]) - 0.5)
            error_v = abs(float(center_uv[1]) - 0.5)
            if (
                error_u <= self.visual_center_tolerance_u
                and error_v <= self.visual_center_tolerance_v
            ):
                return True

        return False

    def log_waiting_for_pickup(
        self,
        visual_lock,
        nearest_ball_name,
        nearest_horizontal_distance,
        nearest_vertical_distance,
    ):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_wait_log_ns < int(1e9):
            return

        if nearest_ball_name is None:
            message = (
                f'Waiting to collect {self.target_color} ball: '
                'no color-matched ball pose available yet.'
            )
        else:
            message = (
                f'Waiting to collect {self.target_color} ball: '
                f'nearest={nearest_ball_name}, '
                f'xy={nearest_horizontal_distance:.2f} m, '
                f'dz={nearest_vertical_distance:.2f} m, '
                f'xy_limit={self.attach_xy_threshold_m:.2f} m, '
                f'visual_xy_limit={self.visual_attach_xy_threshold_m:.2f} m, '
                f'visual_lock={visual_lock}'
            )

        self.get_logger().info(message)
        self.last_wait_log_ns = now_ns

    def publish_state(self):
        with self.state_lock:
            collected_colors = sorted(
                {
                    record['color']
                    for record in self.collected_ball_records
                    if record.get('color')
                }
            )
            last_collected_color = None
            if self.collected_ball_records:
                last_collected_color = self.collected_ball_records[-1]['color']

            payload = {
                'enabled': bool(self.acquire_enabled),
                'target_color': self.target_color,
                'holding_ball': bool(self.collected_ball_records),
                'has_collected_ball': bool(self.collected_ball_records),
                'last_collected_color': last_collected_color,
                'collected_colors': collected_colors,
                'virtual_payload_colors': collected_colors,
                'payload_count': len(self.collected_ball_records),
                'collected_balls': list(self.collected_ball_records),
                'available_ball_count': len(self.ball_poses),
            }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.state_publisher.publish(msg)

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
