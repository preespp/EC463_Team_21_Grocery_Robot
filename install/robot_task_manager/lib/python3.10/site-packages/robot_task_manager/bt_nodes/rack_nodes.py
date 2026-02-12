import py_trees

class RepositionRackToGoalLevel(py_trees.behaviour.Behaviour):
    """
    If goal_key is str: read from bb.<goal_key>
    If goal_key is int: treat as constant level
    """
    def __init__(self, goal_key):
        super().__init__(f"RepositionRackToGoalLevel[{goal_key}]")
        self.goal_key = goal_key
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        if isinstance(self.goal_key, str):
            level = getattr(self.bb, self.goal_key, None)
        else:
            level = self.goal_key

        if level is None:
            return py_trees.common.Status.FAILURE

        # TODO: call rack service
        return py_trees.common.Status.SUCCESS
