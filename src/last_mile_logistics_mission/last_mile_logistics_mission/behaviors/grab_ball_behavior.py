import py_trees


class GrabBall(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        node,
        color,
        pickup_height_m,
        altitude_tolerance_m,
    ):
        super().__init__(name)
        self.node = node
        self.color = color.lower()
        self.pickup_height_m = max(float(pickup_height_m), 0.1)
        self.altitude_tolerance_m = max(float(altitude_tolerance_m), 0.05)
        self.target_world_z = None
        self.pickup_aligned_since_ns = None
        self.grabber_enabled = False
        self.missing_since_ns = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='home_position',
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key='current_position',
            access=py_trees.common.Access.READ,
        )

    def initialise(self):
        home_position = self.blackboard.home_position
        current_position = self.blackboard.current_position
        self.pickup_aligned_since_ns = None
        self.grabber_enabled = False
        self.missing_since_ns = None
        if home_position is None or current_position is None:
            self.target_world_z = None
            return

        self.target_world_z = home_position[2] - self.pickup_height_m
        self.node.get_logger().info(
            f'Descending to pickup height for the {self.color} ball.'
        )
        self.node.publish_ball_mode(f'hover_{self.color}')
        self.node.set_ball_grabber_enabled(False, self.color)
        self.node.publish_target(
            x=current_position[0],
            y=current_position[1],
            z=self.target_world_z,
        )

    def update(self):
        if self.node.has_collected_ball(self.color):
            return py_trees.common.Status.SUCCESS

        current_position = self.blackboard.current_position
        if self.target_world_z is None or current_position is None:
            return py_trees.common.Status.FAILURE

        altitude_ready = (
            abs(current_position[2] - self.target_world_z)
            <= self.altitude_tolerance_m
        )
        if not altitude_ready:
            self.pickup_aligned_since_ns = None
            return py_trees.common.Status.RUNNING

        detection = self.node.get_detection(self.color)
        if detection is None:
            now_ns = self.node.get_clock().now().nanoseconds
            if self.missing_since_ns is None:
                self.missing_since_ns = now_ns
            elif now_ns - self.missing_since_ns >= int(
                self.node.approach_lost_timeout_s * 1e9
            ):
                self.node.get_logger().warn(
                    f'Lost {self.color} ball during final descent. Returning to search altitude.',
                    throttle_duration_sec=2.0,
                )
                return py_trees.common.Status.FAILURE
            self.pickup_aligned_since_ns = None
            self.node.get_logger().info(
                f'Pickup altitude reached for the {self.color} ball. '
                'Waiting for precise visual alignment.',
                throttle_duration_sec=1.0,
            )
            return py_trees.common.Status.RUNNING

        self.missing_since_ns = None
        center_uv = detection.get('center_uv', [0.5, 0.5])
        if len(center_uv) != 2:
            self.pickup_aligned_since_ns = None
            return py_trees.common.Status.RUNNING

        error_u = abs(0.5 - float(center_uv[0]))
        error_v = abs(0.5 - float(center_uv[1]))
        pickup_aligned = (
            error_u <= self.node.pickup_center_tolerance_u
            and error_v <= self.node.pickup_center_tolerance_v
        )
        now_ns = self.node.get_clock().now().nanoseconds

        if pickup_aligned:
            if self.pickup_aligned_since_ns is None:
                self.pickup_aligned_since_ns = now_ns
            elif (
                not self.grabber_enabled
                and now_ns - self.pickup_aligned_since_ns
                >= int(self.node.pickup_center_hold_s * 1e9)
            ):
                self.grabber_enabled = True
                self.node.get_logger().info(
                    f'{self.color.capitalize()} ball centered under the drone. '
                    'Enabling grabber for pickup.'
                )
                self.node.set_ball_grabber_enabled(True, self.color)
                return py_trees.common.Status.SUCCESS
        else:
            self.pickup_aligned_since_ns = None

        self.node.get_logger().info(
            f'Pickup align {self.color}: '
            f'center_uv=({center_uv[0]:.3f},{center_uv[1]:.3f}), '
            f'error=({error_u:.3f},{error_v:.3f}), '
            f'altitude_ready={altitude_ready}, '
            f'pickup_aligned={pickup_aligned}',
            throttle_duration_sec=1.0,
        )

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE and not self.grabber_enabled:
            self.node.set_ball_grabber_enabled(False, self.color)
            self.node.publish_ball_mode('stop')
