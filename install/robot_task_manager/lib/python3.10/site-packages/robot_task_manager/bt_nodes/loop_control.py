import py_trees

class HasMoreItems(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__("HasMoreItems")
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        items = getattr(self.bb, "items", [])
        idx = getattr(self.bb, "item_index", 0)
        return py_trees.common.Status.SUCCESS if idx < len(items) else py_trees.common.Status.FAILURE


class AllItemsDone(py_trees.behaviour.Behaviour):
    """
    Used as the "exit" path in the loop selector.
    Always SUCCESS so the loop body can end gracefully.
    """
    def __init__(self):
        super().__init__("AllItemsDone")

    def update(self):
        return py_trees.common.Status.SUCCESS
