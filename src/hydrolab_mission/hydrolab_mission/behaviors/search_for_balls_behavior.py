import py_trees

class SearchForBalls(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, forward_distance, search_z):
        super().__init__(name)
        self.node = node
        self.forward_distance = forward_distance
        self.search_z = search_z

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="home_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="gimbal_status", access=py_trees.common.Access.READ)
        self.target_world_z = None

    def initialise(self):
        self.target_world_z = self.blackboard.home_position[2] + self.search_z
        self.node.get_logger().info(f"Starting Search Pattern: Moving to search location...")
        self.node.set_gimbal_tracker_enabled(True)
        # 1. Fly to the provided location
        self.node.publish_target(
            x=self.blackboard.home_position[0],
            y=self.blackboard.home_position[1] + self.forward_distance,  
            z=self.target_world_z
        )
        
        # 2. Initiate Gimbal Sequence by publishing COMMAND_SEARCH
        self.node.get_logger().info("Sending COMMAND_SEARCH to gimbal...")
        self.node.send_gimbal_command("COMMAND_SEARCH")

    def update(self):
        # 3. Check if Gimbal confirms it achieved lock-on automatically from its end
        if self.blackboard.gimbal_status == "STATUS_LOCKED":
            # Wait to ensure lock is stable before instantly skipping the search state
            self.node.get_logger().info("Gimbal asserts lock on ball. Stabilizing...")
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(f"Moving to location & gimbal state is '{self.blackboard.gimbal_status}'...", throttle_duration_sec=2.0)
        
        # Otherwise, keep flying towards the target and scanning
        return py_trees.common.Status.RUNNING
