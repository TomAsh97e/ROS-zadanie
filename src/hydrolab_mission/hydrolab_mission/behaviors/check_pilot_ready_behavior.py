import py_trees


class CheckPilotReady(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key="pilot_ready",
            access=py_trees.common.Access.READ,
        )

    def update(self):
        if self.blackboard.exists("pilot_ready") and bool(self.blackboard.pilot_ready):
            self.node.get_logger().info(
                "Pilot ready for mission commands.",
                throttle_duration_sec=2.0,
            )
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            "Waiting for pilot readiness...",
            throttle_duration_sec=2.0,
        )
        return py_trees.common.Status.RUNNING
