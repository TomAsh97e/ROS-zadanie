from enum import Enum, auto


class Status(Enum):
    INVALID = auto()
    FAILURE = auto()
    SUCCESS = auto()
    RUNNING = auto()


class Access(Enum):
    READ = auto()
    WRITE = auto()

