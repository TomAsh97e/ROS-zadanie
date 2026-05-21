import py_trees


class VerifyBallGrabbed(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, color, timeout_s):
        super().__init__(name)
        self.node = node
        self.color = color.lower()
        self.timeout_s = max(float(timeout_s), 0.1)
        self.deadline_ns = 0

    def initialise(self):
        self.deadline_ns = (
            self.node.get_clock().now().nanoseconds
            + int(self.timeout_s * 1e9)
        )

    def update(self):
        if self.node.has_collected_ball(self.color):
            self.node.get_logger().info(
                f'Confirmed collection of the {self.color} ball.'
            )
            return py_trees.common.Status.SUCCESS

        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns >= self.deadline_ns:
            self.node.get_logger().warn(
                f'Timed out while waiting for {self.color} ball collection.'
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.node.set_ball_grabber_enabled(False)
            self.node.publish_ball_mode('stop')
