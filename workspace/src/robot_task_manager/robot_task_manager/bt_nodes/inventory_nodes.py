import py_trees
import requests

SERVER = "http://localhost:3000"


class ChangeInventory(py_trees.behaviour.Behaviour):
    """
    Uses bb.current_item and decrements inventory in PostgreSQL API.
    """
    def __init__(self, bb, mode):
        super().__init__("ChangeInventory")
        self.bb = bb
        self.mode = mode

    def update(self):
        item = getattr(self.bb, "current_item", None)
        if item is None:
            return py_trees.common.Status.FAILURE

        product_id = str(getattr(item, "product_id", ""))
        qty = int(getattr(self.bb, "num_current_item", 0))
        if not product_id or qty <= 0:
            return py_trees.common.Status.FAILURE

        try:
            r = requests.post(
                f"{SERVER}/api/inventory/decrement",
                json={"product_id": product_id, "qty": qty},
                timeout=1.0,
            )
            if not r.ok:
                return py_trees.common.Status.FAILURE
        except requests.exceptions.RequestException:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS
