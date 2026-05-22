import py_trees


class ReturnToLaunch(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, target_hover_altitude_m, position_tolerance_m):
        super().__init__(name)
        self.node = node
        self.target_hover_altitude_m = float(target_hover_altitude_m)
        self.position_tolerance_m = max(float(position_tolerance_m), 0.05)

        self.target_x_m = None
        self.target_y_m = None
        self.target_world_z = None

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key="home_position",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key="current_position",
            access=py_trees.common.Access.READ,
        )

    def initialise(self):
        home_position = self.blackboard.home_position
        if home_position is None:
            self.target_x_m = None
            self.target_y_m = None
            self.target_world_z = None
            return

        self.target_x_m = float(home_position[0])
        self.target_y_m = float(home_position[1])
        # Command landing at launch coordinates (ground/home altitude).
        self.target_world_z = float(home_position[2])
        self.node.get_logger().info(
            "Returning to launch and landing at "
            f"({self.target_x_m:.1f}, {self.target_y_m:.1f}, {self.target_world_z:.2f})..."
        )
        self.node.publish_target(
            x=self.target_x_m,
            y=self.target_y_m,
            z=self.target_world_z,
        )

    def update(self):
        if (
            self.target_x_m is None
            or self.target_y_m is None
            or self.target_world_z is None
        ):
            return py_trees.common.Status.RUNNING

        current_position = self.blackboard.current_position
        if current_position is None:
            return py_trees.common.Status.RUNNING

        self.node.publish_target(
            x=self.target_x_m,
            y=self.target_y_m,
            z=self.target_world_z,
        )

        dx = float(current_position[0]) - self.target_x_m
        dy = float(current_position[1]) - self.target_y_m
        dz = float(current_position[2]) - self.target_world_z
        distance_to_target = (dx * dx + dy * dy + dz * dz) ** 0.5

        if distance_to_target <= self.position_tolerance_m:
            self.node.get_logger().info("Reached launch waypoint.")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
