import py_trees


class HasWaterSample(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, pool_color):
        super().__init__(name)
        self.node = node
        self.pool_color = pool_color

    def update(self):
        # TODO: Check whether a water sample is already onboard.
        return py_trees.common.Status.FAILURE
