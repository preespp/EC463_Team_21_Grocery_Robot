import py_trees

class SetCurrentItem(py_trees.behaviour.Behaviour):
    """
    Pops next item from bb.items using bb.item_index.
    Also sets typical per-item goals for reuse in other nodes:
      - bb.rack_goal (from item.shelf_level)
      - bb.nav_goal  (placeholder: should be computed from aisle/rack later)
      - bb.shelf_pose (placeholder)
    """
    def __init__(self):
        super().__init__("SetCurrentItem")
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        items = getattr(self.bb, "items", [])
        idx = getattr(self.bb, "item_index", 0)

        if idx >= len(items):
            return py_trees.common.Status.FAILURE

        item = items[idx]
        self.bb.current_item = item
        self.bb.item_index = idx + 1

        # TODO: Replace with real map look up from database
        self.bb.nav_goal = getattr(self.bb, "nav_goal", None)
        self.bb.shelf_pose = getattr(self.bb, "shelf_pose", None)
        self.bb.rack_goal = int(getattr(item, "shelf_level", 0))

        return py_trees.common.Status.SUCCESS


class SetHome(py_trees.behaviour.Behaviour):
    """
    Sets bb.nav_goal = bb.home_pose for NavigateToGoalPose reuse.
    """
    def __init__(self):
        super().__init__("SetHome")
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        home = getattr(self.bb, "home_pose", None)
        if home is None:
            return py_trees.common.Status.FAILURE
        self.bb.nav_goal = home
        return py_trees.common.Status.SUCCESS
