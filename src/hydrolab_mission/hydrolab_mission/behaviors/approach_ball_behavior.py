import py_trees

class ApproachBall(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, color):
        super().__init__(name)
        self.node = node
        self.color = color.lower()
        self.centered_since_ns = None
        self.missing_since_ns = None
        self.holding_center = False

    def initialise(self):
        self.centered_since_ns = None
        self.missing_since_ns = None
        self.holding_center = False
        self.node.publish_ball_mode(f'hover_{self.color}')
        self.node.get_logger().info(
            f'Approaching {self.color} ball with continuous visual tracking...'
        )

    def update(self):
        detection = self.node.get_detection(self.color)
        now_ns = self.node.get_clock().now().nanoseconds

        if detection is None:
            if self.missing_since_ns is None:
                self.missing_since_ns = now_ns

            if now_ns - self.missing_since_ns >= int(
                self.node.approach_lost_timeout_s * 1e9
            ):
                self.node.get_logger().warn(
                    f'Lost {self.color} ball during approach. Restarting search.',
                    throttle_duration_sec=2.0,
                )
                return py_trees.common.Status.FAILURE

            self.centered_since_ns = None
            return py_trees.common.Status.RUNNING

        self.missing_since_ns = None

        center_uv = detection.get('center_uv', [0.5, 0.5])
        if len(center_uv) != 2:
            self.centered_since_ns = None
            return py_trees.common.Status.RUNNING

        error_u = abs(0.5 - float(center_uv[0]))
        error_v = abs(0.5 - float(center_uv[1]))
        centered = (
            error_u <= self.node.approach_center_tolerance_u
            and error_v <= self.node.approach_center_tolerance_v
        )

        if centered:
            if self.centered_since_ns is None:
                self.centered_since_ns = now_ns
            elif (
                not self.holding_center
                and now_ns - self.centered_since_ns
                >= int(self.node.approach_center_hold_s * 1e9)
            ):
                self.holding_center = True
                self.node.get_logger().info(
                    f'{self.color.capitalize()} ball centered. Holding stable target.'
                )
        else:
            self.centered_since_ns = None
            self.holding_center = False

        self.node.get_logger().info(
            f'Approach {self.color}: '
            f'center_uv=({center_uv[0]:.3f},{center_uv[1]:.3f}), '
            f'error=({error_u:.3f},{error_v:.3f}), '
            f'holding={self.holding_center}',
            throttle_duration_sec=1.0,
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.node.publish_ball_mode('stop')
