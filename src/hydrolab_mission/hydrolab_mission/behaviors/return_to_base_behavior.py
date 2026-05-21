import py_trees


class ReturnToBase(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # TODO: Fly straight back to base and deliver the sample.
        return py_trees.common.Status.SUCCESS
