import collections.abc


class MovingWindow(collections.abc.MutableSequence):
    length: int

    def __init__(self, length: int, *args):
        self.length = length
        self.list = list()
        self.extend(list(args))

    def is_window_full(self):
        return len(self.list) >= self.length

    def __len__(self):
        return len(self.list)

    def __getitem__(self, i):
        return self.list[i]

    def __delitem__(self, i):
        del self.list[i]

    def __setitem__(self, i, v):
        self.list[i] = v

    def insert(self, i, v):
        if self.is_window_full():
            del self.list[0]
            self.list.insert(i - 1, v)
        else:
            self.list.insert(i, v)

    def __str__(self):
        return str(self.list)
