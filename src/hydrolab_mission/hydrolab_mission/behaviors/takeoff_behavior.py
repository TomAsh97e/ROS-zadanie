import py_trees

class Takeoff(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, target_z):
        super().__init__(name)
        self.node = node
        self.target_z = target_z
        self.altitude_tolerance = 0.3
        self.target_world_z = None
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="home_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_position", access=py_trees.common.Access.READ)

    def initialise(self):
        self.target_world_z = self.blackboard.home_position[2] + self.target_z
        self.node.get_logger().info(f"Initiating Takeoff to {self.target_z}m...")
        self.node.publish_target(
            x=self.blackboard.home_position[0],
            y=self.blackboard.home_position[1],
            z=self.target_world_z
        )

    def update(self):
        if self.target_world_z is None:
            return py_trees.common.Status.FAILURE

        current_z = self.blackboard.current_position[2]
        if abs(current_z - self.target_world_z) < self.altitude_tolerance:
            self.node.get_logger().info("Takeoff SUCCESS. Altitude reached.")
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING
