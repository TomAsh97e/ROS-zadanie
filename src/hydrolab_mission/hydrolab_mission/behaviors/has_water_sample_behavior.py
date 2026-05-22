import py_trees


class HasWaterSample(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, pool_color):
        super().__init__(name)
        self.node = node
        self.pool_color = pool_color

    def update(self):
        # Temporary test switch controlled by mission parameter.
        if bool(getattr(self.node, 'assume_sample_onboard', False)):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
