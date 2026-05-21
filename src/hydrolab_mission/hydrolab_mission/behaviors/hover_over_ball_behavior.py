import py_trees


class HoverOverBall(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, color):
        super().__init__(name)
        self.node = node
        self.color = color.lower()

    def initialise(self):
        self.node.set_gimbal_tracker_enabled(False)
        self.node.send_gimbal_command('COMMAND_EARTH_NADIR')
        self.node.publish_approach_contract({
            'active': False,
            'tracking_mode': 'hover',
            'target': {
                'kind': 'object',
                'label': 'ball',
                'variant': self.color,
                'source': 'vision/ball_detector',
            },
            'constraints': {
                'require_lock': False,
            },
        })
        self.node.publish_hover_contract({
            'active': True,
            'tracking_mode': 'hover',
            'target': {
                'kind': 'object',
                'label': 'ball',
                'variant': self.color,
                'source': 'vision/ball_detector',
            },
            'constraints': {
                'require_lock': False,
            },
        })

    def update(self):
        # TODO: Implement fine hover/centering phase.
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
            self.node.publish_hover_contract({
                'active': False,
                'tracking_mode': 'idle',
                'target': {},
                'constraints': {},
            })
