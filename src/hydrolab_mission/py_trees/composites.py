from .behaviour import Behaviour
from .common import Status


class Composite(Behaviour):
    def __init__(self, name='Composite', memory=False):
        super().__init__(name)
        self.memory = memory
        self.children = []
        self.current_index = 0

    def add_child(self, child):
        self.children.append(child)

    def add_children(self, children):
        self.children.extend(children)

    def initialise(self):
        if not self.memory:
            self.current_index = 0


class Sequence(Composite):
    def update(self):
        start_index = self.current_index if self.memory else 0

        for index in range(start_index, len(self.children)):
            child = self.children[index]
            child_status = child.tick()

            if child_status == Status.SUCCESS:
                continue

            if child_status == Status.RUNNING:
                if self.memory:
                    self.current_index = index
                return Status.RUNNING

            self.current_index = 0
            return Status.FAILURE

        self.current_index = 0
        return Status.SUCCESS


class Selector(Composite):
    def update(self):
        start_index = self.current_index if self.memory else 0

        for index in range(start_index, len(self.children)):
            child = self.children[index]
            child_status = child.tick()

            if child_status == Status.FAILURE:
                continue

            if child_status == Status.RUNNING:
                if self.memory:
                    self.current_index = index
                return Status.RUNNING

            self.current_index = 0
            return Status.SUCCESS

        self.current_index = 0
        return Status.FAILURE

