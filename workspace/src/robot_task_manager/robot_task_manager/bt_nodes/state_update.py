import py_trees


def _to_float(v, default=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float(default)


def _to_int(v, default=0):
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return int(default)


class SetCurrentItem(py_trees.behaviour.Behaviour):
    """
    Pops next item from bb.items using bb.item_index.
    Also sets typical per-item goals for reuse in other nodes:
      - bb.rack_goal (from item z / shelf_level)
      - bb.nav_goal  (from item x/y)
      - bb.shelf_pose (placeholder)
    """
    def __init__(self, bb):
        super().__init__("SetCurrentItem")
        self.bb = bb

    def update(self):
        items = getattr(self.bb, "items", [])
        idx = getattr(self.bb, "item_index", 0)
        total_items = len(items)

        if idx >= total_items:
            return py_trees.common.Status.FAILURE

        item = items[idx]
        self.bb.current_item = item
        self.bb.item_index = idx + 1
        # Order items are ROS messages (OrderItem), not dicts.
        self.bb.num_current_item = int(getattr(item, "qty", 0))

        item_name = getattr(item, "name", "unknown")
        print(
            f"Processing item {idx + 1}/{total_items}: "
            f"{item_name} (qty={self.bb.num_current_item})"
        )

        x = _to_float(getattr(item, "aisle", 0.0), 0.0)
        y = _to_float(getattr(item, "rack", 0.0), 0.0)
        z = _to_int(getattr(item, "shelf_level", 0), 0)
        self.bb.nav_goal = (x, y)
        self.bb.rack_goal = z

        return py_trees.common.Status.SUCCESS


class SetHome(py_trees.behaviour.Behaviour):
    """
    Sets bb.nav_goal = bb.home_pose for NavigateToGoalPose reuse.
    """
    def __init__(self, bb):
        super().__init__("SetHome")
        self.bb = bb

    def update(self):
        self.bb.nav_goal = getattr(self.bb, "home_goal", None)
        self.bb.rack_goal = getattr(self.bb, "home_pose", 1)
        self.bb.pose = getattr(self.bb, "home_rack", 1)
        return py_trees.common.Status.SUCCESS
