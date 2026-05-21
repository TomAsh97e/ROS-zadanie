from .common import Status


class Behaviour:
    def __init__(self, name='Behaviour'):
        self.name = name
        self.status = Status.INVALID

    def initialise(self):
        pass

    def update(self):
        return Status.SUCCESS

    def terminate(self, new_status):
        pass

    def tick(self):
        if self.status != Status.RUNNING:
            self.initialise()

        new_status = self.update()

        if new_status != Status.RUNNING:
            self.terminate(new_status)

        self.status = new_status
        return self.status

