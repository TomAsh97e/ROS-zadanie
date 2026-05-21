import py_trees


class HasBall(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, color):
        super().__init__(name)
        self.node = node
        self.color = color

    def update(self):
        if self.node.has_collected_ball(self.color):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
