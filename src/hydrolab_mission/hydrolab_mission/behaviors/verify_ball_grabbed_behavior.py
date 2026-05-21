import py_trees

class VerifyBallGrabbed(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # TODO: Implement logic to verify the ball was physically secured
        return py_trees.common.Status.SUCCESS
