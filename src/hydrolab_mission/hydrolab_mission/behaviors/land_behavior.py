import py_trees
from px4_msgs.msg import VehicleCommand


class Land(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, approach_hover_altitude_m, position_tolerance_m):
        super().__init__(name)
        self.node = node
        self.approach_hover_altitude_m = max(float(approach_hover_altitude_m), 0.5)
        self.position_tolerance_m = max(float(position_tolerance_m), 0.05)

        self.target_x_m = None
        self.target_y_m = None
        self.target_z_m = None
        self.phase = 'approach_launch'
        self._last_landing_wait_log_ns = 0

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key="home_position",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key="current_position",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key="landed",
            access=py_trees.common.Access.READ,
        )

    def initialise(self):
        home_position = self.blackboard.home_position
        if home_position is None:
            self.target_x_m = None
            self.target_y_m = None
            self.target_z_m = None
            return

        self.target_x_m = float(home_position[0])
        self.target_y_m = float(home_position[1])
        self.target_z_m = float(home_position[2]) - self.approach_hover_altitude_m
        self.phase = 'approach_launch'
        self._last_landing_wait_log_ns = 0
        self.node.get_logger().info(
            "Landing phase 1/2: moving to launch hover point "
            f"({self.target_x_m:.1f}, {self.target_y_m:.1f}, {self.target_z_m:.2f})..."
        )
        self.node.publish_landing_active(False)
        # Stop mission-time tracking/vision activity before touchdown.
        self.node.request_ball_detection('stop')
        self.node.publish_ball_mode('idle')
        self.node.set_gimbal_tracker_enabled(False)
        self.node.send_gimbal_command('COMMAND_IDLE')
        self.node.publish_target(
            x=self.target_x_m,
            y=self.target_y_m,
            z=self.target_z_m,
        )

    def update(self):
        if self.target_x_m is None or self.target_y_m is None or self.target_z_m is None:
            return py_trees.common.Status.RUNNING

        if bool(getattr(self.blackboard, "landed", False)):
            self.node.get_logger().info("Landing detector reports landed.")
            return py_trees.common.Status.SUCCESS

        current_position = self.blackboard.current_position
        if current_position is None:
            return py_trees.common.Status.RUNNING

        if self.phase == 'approach_launch':
            self.node.publish_target(
                x=self.target_x_m,
                y=self.target_y_m,
                z=self.target_z_m,
            )

            dx = float(current_position[0]) - self.target_x_m
            dy = float(current_position[1]) - self.target_y_m
            dz = float(current_position[2]) - self.target_z_m
            distance_to_target = (dx * dx + dy * dy + dz * dz) ** 0.5

            if distance_to_target <= self.position_tolerance_m:
                self.node.get_logger().info(
                    "Landing phase 2/2: at launch point, stopping target stream and sending NAV_LAND."
                )
                self.node.publish_landing_active(True)
                self.node.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND,
                )
                self.phase = 'wait_for_touchdown'
            return py_trees.common.Status.RUNNING

        # Phase 2: do not send target setpoints anymore; wait for PX4 landing detector.
        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns - self._last_landing_wait_log_ns >= int(2.0 * 1e9):
            self.node.get_logger().info(
                "Waiting for touchdown confirmation from vehicle_land_detected..."
            )
            self._last_landing_wait_log_ns = now_ns

        return py_trees.common.Status.RUNNING
