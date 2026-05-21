import py_trees


class FlyToPool(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        node,
        forward_distance,
        search_z,
        pool_x_m=None,
        pool_y_m=None,
    ):
        super().__init__(name)
        self.node = node
        self.forward_distance = forward_distance
        self.search_z = search_z
        self.pool_x_m = pool_x_m
        self.pool_y_m = pool_y_m

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="home_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="gimbal_status", access=py_trees.common.Access.READ)
        self.target_world_z = None
        self.target_x = None
        self.target_y = None

    def initialise(self):
        self.target_world_z = self.blackboard.home_position[2] + self.search_z
        if self.pool_x_m is not None and self.pool_y_m is not None:
            target_x = self.pool_x_m
            target_y = self.pool_y_m
            self.node.get_logger().info(
                f'Flying to pool waypoint at ({target_x:.1f}, {target_y:.1f})...'
            )
        else:
            target_x = self.blackboard.home_position[0]
            target_y = self.blackboard.home_position[1] + self.forward_distance
            self.node.get_logger().info('Flying to pool search waypoint (home + forward)...')
        self.target_x = target_x
        self.target_y = target_y
        self.node.set_gimbal_tracker_enabled(True)
        self.node.publish_target(
            x=target_x,
            y=target_y,
            z=self.target_world_z,
        )
        self.node.get_logger().info("Sending COMMAND_SEARCH to gimbal...")
        self.node.send_gimbal_command("COMMAND_SEARCH")

    def update(self):
        if self.blackboard.gimbal_status == "STATUS_LOCKED":
            self.node.get_logger().info(
                'Pool target spotted (gimbal lock). Handing off to raycasted approach...'
            )
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f"Flying to pool waypoint; gimbal state is '{self.blackboard.gimbal_status}'...",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING
