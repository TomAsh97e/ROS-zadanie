from .behaviour import Behaviour
from .common import Status


class Decorator(Behaviour):
    def __init__(self, name, child):
        super().__init__(name)
        self.decorated = child

    def tick(self):
        if self.status != Status.RUNNING:
            self.initialise()

        self.decorated.tick()
        new_status = self.update()

        if new_status != Status.RUNNING:
            self.terminate(new_status)

        self.status = new_status
        return self.status

    def update(self):
        return self.decorated.status


class Inverter(Decorator):
    def update(self):
        status = self.decorated.status
        if status == Status.SUCCESS:
            return Status.FAILURE
        if status == Status.FAILURE:
            return Status.SUCCESS
        return status


class Retry(Decorator):
    def __init__(self, name, child, num_failures):
        super().__init__(name=name, child=child)
        self.num_failures = max(int(num_failures), 1)
        self.failure_count = 0

    def initialise(self):
        if self.status != Status.RUNNING:
            self.failure_count = 0

    def update(self):
        status = self.decorated.status

        if status == Status.SUCCESS:
            self.failure_count = 0
            return Status.SUCCESS

        if status == Status.RUNNING:
            return Status.RUNNING

        self.failure_count += 1
        if self.failure_count >= self.num_failures:
            self.failure_count = 0
            return Status.FAILURE
        return Status.RUNNING

