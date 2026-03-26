import py_trees
from robot_interfaces.msg import Order

def setup_blackboard():
    bb = py_trees.blackboard.Blackboard()

    bb.mode = None

    bb.order: Order | None = None
    bb.order_id_text = None

    bb.items = []
    bb.item_index = 0
    bb.current_item = None
    bb.num_current_item = 0

    # For manipulation

    bb.goal_pose = None 
    # I am not sure what this is for but to add to solve conflict


    bb.home_pose = None # Home Pose for Arm to see the shelf, assign Constant in here
    bb.basket_pose = None # Assign Constant in here
    bb.pose = None # Placeholder for current pose of the arm
    bb.goal_pose = None # Preferred BT arm target pose key for ViperX flow

    # For basket management (3 bottle slots + 1 random slot)
    bb.basket_poses = [
        None,  # Bottle slot 1
        None,  # Bottle slot 2
        None,  # Bottle slot 3
        None,  # Random items slot
    ]
    bb.basket_bottle_count = 0  # Track how many bottles have been picked
    
    # For shelf height (3 levels - to be filled during testing)
    bb.shelf_height = None  # Height parameter for shelf level (will be set to 1, 2, or 3)
    bb.shelf_pose = None  # Shelf pose with x,y empty (only z/height will be used)

    # For object detection results
    bb.detected_object_pose = None  # Pose returned from vision service

    # For rack levels
    bb.current_rack = 1
    bb.rack_goal = 1
    bb.home_rack = 1

    # For navigation
    bb.nav_goal = (0.0, 0.0, 0.0)
    bb.home_goal = (0.0, 0.0, 0.0)
    bb.slot_id = None
    bb.anchor_id = None
    bb.rack_id = None
    bb.semantic_id = None
    bb.semantic_target_label = None
    bb.nav_goal_source = None

    return bb
