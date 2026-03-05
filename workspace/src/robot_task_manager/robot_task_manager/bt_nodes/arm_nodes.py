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

        # Set the new pose on the blackboard for robotics arm
        self.bb.pose = None # Pose to set after IK from camera frame
        return py_trees.common.Status.SUCCESS

class MoveGripper(py_trees.behaviour.Behaviour):
    """
    command: "open" or "close"
    """
    def __init__(self, command: str):
        super().__init__(f"MoveGripper[{command}]")
        self.command = command

    def update(self):
        if self.command not in ("open", "close"):
            return py_trees.common.Status.FAILURE

        # TODO: call gripper service/action
        return py_trees.common.Status.SUCCESS
