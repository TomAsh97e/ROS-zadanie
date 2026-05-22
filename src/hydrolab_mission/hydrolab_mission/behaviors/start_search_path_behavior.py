import py_trees


class StartSearchPath(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # Placeholder behavior: search path logic will be added later.
        return py_trees.common.Status.SUCCESS
