class BehaviourTree:
    def __init__(self, root):
        self.root = root

    def tick(self):
        return self.root.tick()

