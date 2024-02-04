class EnvCounter:
    def __init__(self):
        self.counter = 0

    def count_env(self):
        self.counter += 1

    def get_current_counter(self):
        return self.counter
