import py_trees

class GrabBall(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # TODO: Implement logic to actuate grabber
        return py_trees.common.Status.SUCCESS
