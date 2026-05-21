import py_trees

class CheckTimeLimit(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # TODO: Implement time check logic
        # Returns SUCCESS if < 9.5 mins (time is within limits)
        return py_trees.common.Status.SUCCESS
