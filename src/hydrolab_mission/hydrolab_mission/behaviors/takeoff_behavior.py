import py_trees

class Takeoff(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, target_z, start_timeout_s=4.0, min_progress_m=0.2):
        super().__init__(name)
        self.node = node
        self.target_z = target_z
        self.start_timeout_s = max(float(start_timeout_s), 0.5)
        self.min_progress_m = max(float(min_progress_m), 0.01)
        self.altitude_tolerance = 0.3
        self.target_world_z = None
        self.start_time_ns = None
        self.initial_distance_to_target = None
        self.takeoff_started = False
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="home_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_position", access=py_trees.common.Access.READ)

    def initialise(self):
        self.target_world_z = self.blackboard.home_position[2] + self.target_z
        self.start_time_ns = self.node.get_clock().now().nanoseconds
        self.takeoff_started = False
        current_z = self.blackboard.current_position[2]
        self.initial_distance_to_target = abs(current_z - self.target_world_z)
        self.node.get_logger().info(f"Initiating Takeoff to {self.target_z}m...")
        self.node.publish_target(
            x=self.blackboard.home_position[0],
            y=self.blackboard.home_position[1],
            z=self.target_world_z
        )

    def update(self):
        if self.target_world_z is None:
            return py_trees.common.Status.FAILURE
        if self.blackboard.current_position is None:
            return py_trees.common.Status.RUNNING

        # Keep republishing to avoid startup race with late subscribers.
        self.node.publish_target(
            x=self.blackboard.home_position[0],
            y=self.blackboard.home_position[1],
            z=self.target_world_z
        )

        current_z = self.blackboard.current_position[2]
        if abs(current_z - self.target_world_z) < self.altitude_tolerance:
            self.node.get_logger().info("Takeoff SUCCESS. Altitude reached.")
            return py_trees.common.Status.SUCCESS

        distance_to_target = abs(current_z - self.target_world_z)
        progress_m = self.initial_distance_to_target - distance_to_target
        if progress_m >= self.min_progress_m:
            if not self.takeoff_started:
                self.node.get_logger().info(
                    f"Takeoff started (progress {progress_m:.2f} m)."
                )
                self.takeoff_started = True
            return py_trees.common.Status.RUNNING

        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns - self.start_time_ns >= int(self.start_timeout_s * 1e9):
            self.node.get_logger().warn(
                "Takeoff did not start within timeout. Failing this attempt."
            )
            return py_trees.common.Status.FAILURE
            
        return py_trees.common.Status.RUNNING
