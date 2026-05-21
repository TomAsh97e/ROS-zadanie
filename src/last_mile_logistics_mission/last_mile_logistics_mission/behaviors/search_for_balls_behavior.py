import py_trees


class SearchForBalls(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        node,
        color,
        forward_distance,
        lateral_offset,
        search_z,
        position_tolerance_m,
        downward_scan_hold_s,
        early_detection_distance_m,
    ):
        super().__init__(name)
        self.node = node
        self.color = color.lower()
        self.forward_distance = forward_distance
        self.lateral_offset = float(lateral_offset)
        self.search_z = search_z
        self.position_tolerance_m = max(float(position_tolerance_m), 0.05)
        self.downward_scan_hold_s = max(float(downward_scan_hold_s), 0.0)
        self.early_detection_distance_m = max(float(early_detection_distance_m), 0.0)

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key='home_position',
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key='current_position',
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key='gimbal_status',
            access=py_trees.common.Access.READ,
        )
        self.target_world_z = None
        self.search_x = None
        self.search_y = None
        self.downward_scan_started = False
        self.downward_scan_deadline_ns = 0
        self.search_started = False
        self.early_detection_engaged = False

    def initialise(self):
        self.target_world_z = self.blackboard.home_position[2] + self.search_z
        self.search_x = self.blackboard.home_position[0] + self.lateral_offset
        self.search_y = self.blackboard.home_position[1] + self.forward_distance
        self.downward_scan_started = False
        self.downward_scan_deadline_ns = 0
        self.search_started = False
        self.early_detection_engaged = False

        self.node.get_logger().info(
            f'Flying to the {self.color} search point before enabling ball search.'
        )
        self.node.publish_target(
            x=self.search_x,
            y=self.search_y,
            z=self.target_world_z,
        )

    def update(self):
        current_position = self.blackboard.current_position
        if current_position is None or self.target_world_z is None:
            return py_trees.common.Status.RUNNING

        position_error_xy = (
            (current_position[0] - self.search_x) ** 2
            + (current_position[1] - self.search_y) ** 2
        ) ** 0.5
        altitude_ready = abs(current_position[2] - self.target_world_z) <= 0.25
        at_search_point = (
            position_error_xy <= self.position_tolerance_m
            and altitude_ready
        )

        if not self.search_started:
            if not at_search_point:
                if (
                    self.color == 'blue'
                    and not self.early_detection_engaged
                    and position_error_xy <= self.early_detection_distance_m
                ):
                    self.early_detection_engaged = True
                    self.node.get_logger().info(
                        'Approaching the blue search point. Starting early downward blue detection.'
                    )
                    self.node.request_ball_detection(self.color)
                    self.node.hold_gimbal_down()

                self.node.get_logger().info(
                    f'Flying to search point for the {self.color} ball: '
                    f'xy_error={position_error_xy:.2f} m, '
                    f'altitude_ready={altitude_ready}',
                    throttle_duration_sec=1.0,
                )
                return py_trees.common.Status.RUNNING

            detection = self.node.get_detection(self.color)
            now_ns = self.node.get_clock().now().nanoseconds

            if detection is not None:
                if self.early_detection_engaged:
                    self.node.get_logger().info(
                        f'Detected the {self.color} ball immediately on arrival at the search point.'
                    )
                else:
                    self.node.get_logger().info(
                        f'Detected the {self.color} ball directly below the search point.'
                    )
                return py_trees.common.Status.SUCCESS

            if not self.downward_scan_started:
                self.downward_scan_started = True
                self.downward_scan_deadline_ns = (
                    now_ns + int(self.downward_scan_hold_s * 1e9)
                )
                self.node.get_logger().info(
                    f'Search point reached for the {self.color} ball. Looking straight down first.'
                )
                if not self.early_detection_engaged:
                    self.node.request_ball_detection(self.color)
                    self.node.hold_gimbal_down()
                return py_trees.common.Status.RUNNING

            if now_ns < self.downward_scan_deadline_ns:
                self.node.get_logger().info(
                    f'Looking straight down for the {self.color} ball...',
                    throttle_duration_sec=1.0,
                )
                return py_trees.common.Status.RUNNING

            self.search_started = True
            self.node.get_logger().info(
                f'No {self.color} ball directly below. Enabling gimbal search sweep.'
            )
            self.node.request_ball_detection(self.color)
            self.node.get_logger().info('Sending COMMAND_SEARCH to gimbal...')
            self.node.send_gimbal_command('COMMAND_SEARCH')

        if self.blackboard.gimbal_status == 'STATUS_LOCKED':
            self.node.get_logger().info(
                'Gimbal asserts lock on ball. Stabilizing...'
            )
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f"Searching for the {self.color} ball, gimbal state='{self.blackboard.gimbal_status}'...",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING
