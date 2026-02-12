import py_trees

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
