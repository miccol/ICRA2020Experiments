class Node():
    """A node class for A* """

    def __init__(self, parent=None, position=None, action_from_parent = None):
        self.parent = parent
        self.action_from_parent = action_from_parent
        self.position = position # //rename to value

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position