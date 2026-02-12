import py_trees

class RepositionArmToGoalPose(py_trees.behaviour.Behaviour):
    """
    Reads a pose from bb.<goal_key> and commands arm.
    """
    def __init__(self, goal_key: str):
        super().__init__(f"RepositionArmToGoalPose[{goal_key}]")
        self.goal_key = goal_key
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        pose = getattr(self.bb, self.goal_key, None)
        if pose is None:
            return py_trees.common.Status.FAILURE

        # TODO: Action Client Call for Arm
        return py_trees.common.Status.SUCCESS


class VerifyPosition(py_trees.behaviour.Behaviour):
    """
    Placeholder for vision/pose verification (service later).
    """
    def __init__(self):
        super().__init__("VerifyPosition")

    def update(self):
        # TODO: Vision Services
        return py_trees.common.Status.SUCCESS


class AdjustOrientation(py_trees.behaviour.Behaviour):
    """
    Placeholder for micro adjustment logic (service/action later).
    """
    def __init__(self):
        super().__init__("AdjustOrientation")

    def update(self):
        # TODO: Robotics Arm Actions
        return py_trees.common.Status.SUCCESS
