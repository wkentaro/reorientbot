class StaticDict:
    def __init__(self, value):
        self._value = value

    def get(self, key, default=None):
        return self._value
