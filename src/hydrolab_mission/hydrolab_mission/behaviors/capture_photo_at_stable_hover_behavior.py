import py_trees


class CapturePhotoAtStableHover(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, pool_color):
        super().__init__(name)
        self.node = node
        self.pool_color = pool_color.lower()
        self.started_at_ns = None

    def initialise(self):
        self.started_at_ns = self.node.get_clock().now().nanoseconds
        self.node.set_gimbal_tracker_enabled(False)
        self.node.send_gimbal_command('COMMAND_EARTH_NADIR')
        self.node.publish_hover_contract(
            {
                'active': False,
                'tracking_mode': 'idle',
                'target': {},
                'constraints': {},
            }
        )
        self.node.publish_approach_contract(
            {
                'active': True,
                'tracking_mode': 'descent',
                'target': {
                    'kind': 'object',
                    'label': 'pool',
                    'variant': self.pool_color,
                    'source': 'vision/ball_detector',
                },
                'constraints': {
                    'require_lock': False,
                    'target_altitude_m': float(
                        self.node.capture_photo_hover_altitude_m
                    ),
                },
            }
        )

    def update(self):
        now_ns = self.node.get_clock().now().nanoseconds
        if self.started_at_ns is None:
            self.started_at_ns = now_ns
        elapsed_s = (now_ns - self.started_at_ns) / 1e9
        if elapsed_s >= float(self.node.photo_taking_hover_timeout_s):
            self.node.get_logger().warn(
                f'Capture photo hover timeout reached for {self.pool_color} '
                f'({elapsed_s:.1f}s >= {self.node.photo_taking_hover_timeout_s:.1f}s); '
                'taking photo and returning success.',
            )
            self.take_photo()
            return py_trees.common.Status.SUCCESS

        home_position = self.node.blackboard.home_position
        current_position = self.node.blackboard.current_position
        current_velocity = self.node.blackboard.current_velocity
        if home_position is None or current_position is None or current_velocity is None:
            return py_trees.common.Status.RUNNING

        current_altitude_m = float(home_position[2] - current_position[2])
        target_altitude_m = float(self.node.capture_photo_hover_altitude_m)
        target_position = self.node.latest_target_position
        target_position_ns = int(getattr(self.node, 'latest_target_position_ns', 0))
        if target_position is None or target_position_ns <= 0:
            self.node.get_logger().warn(
                f'No active target setpoint available for {self.pool_color}; '
                'cannot validate stable hover XY position yet.',
                throttle_duration_sec=1.0,
            )
            return py_trees.common.Status.RUNNING

        if now_ns - target_position_ns > int(self.node.approach_lost_timeout_s * 1e9):
            self.node.get_logger().warn(
                f'Stable hover target setpoint is stale for {self.pool_color}; waiting for fresh raycast target.',
                throttle_duration_sec=1.0,
            )
            return py_trees.common.Status.RUNNING

        target_x_m = float(target_position[0])
        target_y_m = float(target_position[1])
        current_x_m = float(current_position[0])
        current_y_m = float(current_position[1])
        current_vx_m_s = float(current_velocity[0])
        current_vy_m_s = float(current_velocity[1])
        current_vz_m_s = float(current_velocity[2])
        tolerance_m = float(self.node.position_tolerance_m)
        velocity_tolerance_xy_m_s = float(
            self.node.capture_photo_velocity_tolerance_xy_m_s
        )
        velocity_tolerance_z_m_s = float(
            self.node.capture_photo_velocity_tolerance_z_m_s
        )

        x_within_tolerance = abs(current_x_m - target_x_m) <= tolerance_m
        y_within_tolerance = abs(current_y_m - target_y_m) <= tolerance_m
        altitude_within_limit = current_altitude_m <= target_altitude_m
        velocity_xy_within_tolerance = (
            abs(current_vx_m_s) <= velocity_tolerance_xy_m_s
            and abs(current_vy_m_s) <= velocity_tolerance_xy_m_s
        )
        velocity_z_within_tolerance = abs(current_vz_m_s) <= velocity_tolerance_z_m_s

        if (
            altitude_within_limit
            and x_within_tolerance
            and y_within_tolerance
            and velocity_xy_within_tolerance
            and velocity_z_within_tolerance
        ):
            self.take_photo()
            self.node.get_logger().info(
                f'Stable hover reached for {self.pool_color}: '
                f'altitude={current_altitude_m:.2f} m '
                f'(target <= {target_altitude_m:.2f} m), '
                f'x={current_x_m:.2f} m (target {target_x_m:.2f} +/- {tolerance_m:.2f}), '
                f'y={current_y_m:.2f} m (target {target_y_m:.2f} +/- {tolerance_m:.2f}), '
                f'vx={current_vx_m_s:.2f} m/s, vy={current_vy_m_s:.2f} m/s '
                f'(tol <= {velocity_tolerance_xy_m_s:.2f}), '
                f'vz={current_vz_m_s:.2f} m/s '
                f'(tol <= {velocity_tolerance_z_m_s:.2f}).'
            )
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f'Stable hover approach active for {self.pool_color}: '
            f'altitude={current_altitude_m:.2f} m, '
            f'target={target_altitude_m:.2f} m, '
            f'x={current_x_m:.2f}/{target_x_m:.2f}, '
            f'y={current_y_m:.2f}/{target_y_m:.2f}, '
            f'vx={current_vx_m_s:.2f} m/s, vy={current_vy_m_s:.2f} m/s, '
            f'vz={current_vz_m_s:.2f} m/s.',
            throttle_duration_sec=1.0,
        )
        return py_trees.common.Status.RUNNING

    def take_photo(self):
        pass

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.node.send_gimbal_command('COMMAND_RESET')
            self.node.publish_approach_contract(
                {
                    'active': False,
                    'tracking_mode': 'idle',
                    'target': {},
                    'constraints': {},
                }
            )
