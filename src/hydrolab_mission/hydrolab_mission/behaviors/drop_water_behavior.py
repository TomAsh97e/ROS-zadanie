import py_trees


class DropWater(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # TODO: Drop collected water into the dedicated base tank.
        return py_trees.common.Status.SUCCESS
