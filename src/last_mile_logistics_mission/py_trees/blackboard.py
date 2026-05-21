_STORE = {}


class Client:
    def __init__(self, name='blackboard'):
        super().__setattr__('_name', name)
        super().__setattr__('_registered_keys', set())

    def register_key(self, key, access):
        self._registered_keys.add(key)

    def exists(self, key):
        return key in _STORE

    def __getattr__(self, item):
        if item in _STORE:
            return _STORE[item]
        raise AttributeError(item)

    def __setattr__(self, key, value):
        if key.startswith('_'):
            super().__setattr__(key, value)
            return
        _STORE[key] = value

