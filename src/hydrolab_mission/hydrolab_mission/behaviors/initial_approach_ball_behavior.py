import py_trees


class InitialApproachBall(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, color):
        super().__init__(name)
        self.node = node
        self.color = color.lower()
        self.completed = False

    def initialise(self):
        self.completed = False
        self.node.set_gimbal_tracker_enabled(True)
        self.node.publish_approach_contract({
            'active': True,
            'tracking_mode': 'initial_approach',
            'target': {
                'kind': 'object',
                'label': 'ball',
                'variant': self.color,
                'source': 'vision/ball_detector',
            },
            'constraints': {
                'require_lock': True,
            },
        })

    def update(self):
        pitch_deg = self.node.current_gimbal_pitch_deg
        if pitch_deg is None:
            return py_trees.common.Status.RUNNING

        pitch_down_deg = -float(pitch_deg)
        lock_persisting = (
            str(self.node.blackboard.gimbal_status).upper() == 'STATUS_LOCKED'
        )

        if pitch_down_deg >= 80.0 and lock_persisting:
            if not self.completed:
                self.node.get_logger().info(
                    f'Initial approach complete for {self.color}: '
                    f'gimbal pitch down {pitch_down_deg:.1f} deg and lock confirmed.'
                )
                self.completed = True
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f'Initial approach active for {self.color}: '
            f'gimbal pitch down {pitch_down_deg:.1f} deg '
            f'(target >= 80.0), lock={lock_persisting}.',
            throttle_duration_sec=1.0,
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status != py_trees.common.Status.RUNNING:
            self.node.publish_approach_contract({
                'active': False,
                'tracking_mode': 'idle',
                'target': {},
                'constraints': {},
            })
