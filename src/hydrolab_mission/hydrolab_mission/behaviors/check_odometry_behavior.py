import py_trees

class CheckOdometry(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node 
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="home_position", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.exists("home_position") and self.blackboard.home_position is not None:
            self.node.get_logger().info("Odometry verified.", throttle_duration_sec=2.0)
            return py_trees.common.Status.SUCCESS
        
        self.node.get_logger().info("Waiting for odometry...", throttle_duration_sec=2.0)
        return py_trees.common.Status.RUNNING