import py_trees

class NavigateToGoalPose(py_trees.behaviour.Behaviour):
    """
    Reusable navigation leaf.
    Reads goal pose from bb.<goal_key>.

    Later you will implement Nav2 action call here.
    For now, it succeeds immediately if goal exists.
    """
    def __init__(self, goal_key: str):
        super().__init__(f"NavigateToGoalPose[{goal_key}]")
        self.goal_key = goal_key
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        goal = getattr(self.bb, self.goal_key, None)
        if goal is None:
            return py_trees.common.Status.FAILURE

        # TODO: replace with Nav2 action client call
        return py_trees.common.Status.SUCCESS
