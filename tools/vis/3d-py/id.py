class id_class:
    def __init__(self):
        self.counter = 0
    def next(self):
        self.counter += 1
        return self.counter

id = id_class()