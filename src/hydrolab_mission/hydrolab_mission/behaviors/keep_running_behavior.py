import py_trees


class KeepRunning(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        return py_trees.common.Status.RUNNING
