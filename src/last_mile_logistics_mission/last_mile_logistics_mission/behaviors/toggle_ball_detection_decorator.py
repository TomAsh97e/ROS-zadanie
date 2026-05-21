import py_trees
from std_msgs.msg import String

class ToggleBallDetection(py_trees.decorators.Decorator):
    """
    Decorator that enables the ball detector for a specific color 
    when the child starts, and disables it when the child finishes.
    """
    def __init__(self, name, child, node, color):
        super().__init__(name=name, child=child)
        self.node = node
        self.color = color

    def update(self):
        # By default in py_trees, Decorators must implement update.
        # We just return the child's status.
        return self.decorated.status

    def initialise(self):
        # SearchForBalls now enables detection only after the drone reaches
        # the search point, so we keep initialise() passive here.
        return

    def terminate(self, new_status):
        # Called when the child finishes (SUCCESS, FAILURE) or is cancelled (INVALID)
        if new_status in [py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE, py_trees.common.Status.INVALID]:
            self.node.get_logger().info(f"Disabling ball detection.")
            self.node.request_ball_detection("stop")
