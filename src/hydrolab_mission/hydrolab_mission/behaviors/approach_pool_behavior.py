import py_trees


class ApproachPool(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, pool_color, final_speed_m_s, final_length_m):
        super().__init__(name)
        self.node = node
        self.pool_color = pool_color.lower()
        self.final_speed_m_s = float(final_speed_m_s)
        self.final_length_m = float(final_length_m)
        self.completed = False

    def initialise(self):
        self.completed = False
        self.node.set_gimbal_tracker_enabled(True)
        self.node.get_logger().info(
            f'Starting raycasted approach over {self.pool_color} pool '
            f'(vision-guided fly-over).'
        )
        self.node.publish_approach_contract({
            'active': True,
            'tracking_mode': 'initial_approach',
            'target': {
                'kind': 'object',
                'label': 'pool',
                'variant': self.pool_color,
                'source': 'vision/ball_detector',
            },
            'constraints': {
                'require_lock': True,
                'final_speed_m_s': self.final_speed_m_s,
                'final_length_m': self.final_length_m,
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
                    f'Pool approach complete for {self.pool_color}: '
                    f'gimbal pitch down {pitch_down_deg:.1f} deg and lock confirmed.'
                )
                self.completed = True
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().info(
            f'Raycasted pool approach active for {self.pool_color}: '
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
