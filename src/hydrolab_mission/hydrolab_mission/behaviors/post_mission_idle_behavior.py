import py_trees


class PostMissionIdle(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def initialise(self):
        self.node.get_logger().info("Mission complete. Holding post-mission idle state.")

    def update(self):
        return py_trees.common.Status.RUNNING
