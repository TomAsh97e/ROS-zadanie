import py_trees

class HasBall(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, color):
        super().__init__(name)
        self.node = node
        self.color = color

    def update(self):
        # TODO: Check if the ball of the specified color is already grabbed
        # Return SUCCESS if already grabbed, otherwise FAILURE
        return py_trees.common.Status.FAILURE
