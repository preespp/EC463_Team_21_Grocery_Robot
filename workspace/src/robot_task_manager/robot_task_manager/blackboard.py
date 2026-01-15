import py_trees
from robot_interfaces.msg import Order

def setup_blackboard():
    bb = py_trees.blackboard.Blackboard()

    bb.mode = None

    bb.order: Order | None = None

    bb.items = []
    bb.item_index = 0
    bb.current_item = None

    bb.shelf_pose = None
    bb.basket_pose = None
    bb.home_pose = None

    return bb
