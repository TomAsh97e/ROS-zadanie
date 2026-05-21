import py_trees


class VerifySampleCollected(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # TODO: Verify the water sample was collected successfully.
        return py_trees.common.Status.SUCCESS
