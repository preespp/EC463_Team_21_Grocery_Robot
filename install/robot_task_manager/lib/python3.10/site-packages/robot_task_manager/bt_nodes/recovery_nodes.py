import py_trees

class RetryGrab(py_trees.behaviour.Behaviour):
    def __init__(self, max_retries: int = 3):
        super().__init__(f"RetryGrab[{max_retries}]")
        self.max_retries = max_retries
        self.attempts = 0

    def update(self):
        self.attempts += 1
        if self.attempts <= self.max_retries:
            # Returning FAILURE makes Selector try first branch again next tick
            return py_trees.common.Status.FAILURE
        # Add condition to check if it's success or not
        return py_trees.common.Status.FAILURE


class RetryPlace(py_trees.behaviour.Behaviour):
    def __init__(self, max_retries: int = 3):
        super().__init__(f"RetryPlace[{max_retries}]")
        self.max_retries = max_retries
        self.attempts = 0

    def update(self):
        self.attempts += 1
        if self.attempts <= self.max_retries:
            return py_trees.common.Status.FAILURE
        # Add condition to check if it's success or not
        return py_trees.common.Status.FAILURE
