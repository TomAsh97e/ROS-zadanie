import py_trees

class ReturnToLaunch(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # TODO: Implement Return To Launch logic
        return py_trees.common.Status.RUNNING
