import py_trees


class DescendOverBall(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, color):
        super().__init__(name)
        self.node = node
        self.color = color.lower()

    def initialise(self):
        self.node.set_gimbal_tracker_enabled(False)
        self.node.send_gimbal_command('COMMAND_EARTH_NADIR')
        self.node.publish_hover_contract({
            'active': False,
            'tracking_mode': 'idle',
            'target': {},
            'constraints': {},
        })
        self.node.publish_approach_contract({
            'active': True,
            'tracking_mode': 'descent',
            'target': {
                'kind': 'object',
                'label': 'ball',
                'variant': self.color,
                'source': 'vision/ball_detector',
            },
            'constraints': {
                'require_lock': False,
                'target_altitude_m': float(self.node.descent_target_altitude_m),
            },
        })

    def update(self):
        home_position = self.node.blackboard.home_position
        current_position = self.node.blackboard.current_position
        if home_position is None or current_position is None:
            return py_trees.common.Status.RUNNING

        current_altitude_m = float(home_position[2] - current_position[2])
        target_altitude_m = float(self.node.descent_target_altitude_m)
        if current_altitude_m <= target_altitude_m:
            self.node.get_logger().info(
                f'Final descent complete for {self.color}: '
                f'altitude={current_altitude_m:.2f} m '
                f'(target <= {target_altitude_m:.2f} m).'
            )
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f'Final descent active for {self.color}: '
            f'altitude={current_altitude_m:.2f} m, '
            f'target={target_altitude_m:.2f} m.',
            throttle_duration_sec=1.0,
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.node.send_gimbal_command('COMMAND_RESET')
            self.node.publish_approach_contract({
                'active': False,
                'tracking_mode': 'idle',
                'target': {},
                'constraints': {},
            })
