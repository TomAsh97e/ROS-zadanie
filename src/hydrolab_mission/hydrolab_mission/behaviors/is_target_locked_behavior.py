import py_trees


class IsTargetLocked(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node

    def update(self):
        if bool(getattr(self.node.blackboard, 'target_detected', False)):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
