import py_trees

class ChangeInventory(py_trees.behaviour.Behaviour):
    """
    Uses bb.current_item and calls inventory update (service later).
    """
    def __init__(self):
        super().__init__("ChangeInventory")
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        item = getattr(self.bb, "current_item", None)
        if item is None:
            return py_trees.common.Status.FAILURE

        # TODO: Use DB Function to change database data
        return py_trees.common.Status.SUCCESS
