import py_trees


class CollectWaterSample(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        # TODO: Actuate sampler and collect 500 ml of water.
        return py_trees.common.Status.SUCCESS
