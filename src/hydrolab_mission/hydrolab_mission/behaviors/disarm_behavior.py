import py_trees
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus


class Disarm(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.last_command_sent_ns = 0
        self.command_period_s = 1.0

        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(
            key="arming_state",
            access=py_trees.common.Access.READ,
        )

    def initialise(self):
        self.last_command_sent_ns = 0

    def update(self):
        arming_state = int(getattr(self.blackboard, "arming_state", 0))
        if arming_state == int(VehicleStatus.ARMING_STATE_DISARMED):
            self.node.get_logger().info("Vehicle status reports disarmed.")
            return py_trees.common.Status.SUCCESS

        now_ns = self.node.get_clock().now().nanoseconds
        if (
            self.last_command_sent_ns == 0
            or now_ns - self.last_command_sent_ns >= int(self.command_period_s * 1e9)
        ):
            self.node.get_logger().info("Sending disarm command...")
            self.node.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0,
            )
            self.last_command_sent_ns = now_ns

        return py_trees.common.Status.RUNNING
