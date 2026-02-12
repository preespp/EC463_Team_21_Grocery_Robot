import py_trees
from robot_interfaces.msg import Order

def setup_blackboard():
    bb = py_trees.blackboard.Blackboard()

    bb.mode = None

    bb.order: Order | None = None

    bb.items = []
    bb.item_index = 0
    bb.current_item = None
    bb.num_current_item = 0

    bb.nav_goal = (0.0, 0.0)
    bb.rack_goal = 1

    # Fill out once we conclude the value of each pose for robotics arm
    bb.shelf_pose = None # Depends on data from database
    bb.basket_pose = None # Assign Constant in here
    bb.home_pose = None # Assign Constant in here

    return bb
