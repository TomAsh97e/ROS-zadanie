import py_trees


class TogglePoolDetection(py_trees.decorators.Decorator):
    """Enable pool detection for a color when the child runs; disable it when done."""

    def __init__(self, name, child, node, pool_color):
        super().__init__(name=name, child=child)
        self.node = node
        self.pool_color = pool_color

    def update(self):
        return self.decorated.status

    def initialise(self):
        self.node.get_logger().info(
            f"Enabling pool detection for color: {self.pool_color}"
        )
        self.node.request_ball_detection(self.pool_color)

    def terminate(self, new_status):
        if new_status in [
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
            py_trees.common.Status.INVALID,
        ]:
            self.node.get_logger().info("Disabling pool detection.")
            self.node.request_ball_detection("stop")
