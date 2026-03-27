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
    # Preset arm states are stored as BT command dictionaries.
    # The corresponding raw joint values live in robot_manipulation config/server parameters.
    bb.home_pose = {"command": "return_arm_pose"}
    bb.basket_pose = {"command": "place_arm_pose"}
    bb.pose = {"command": "startup_arm_pose"}
    bb.goal_pose = None # Preferred BT arm target pose key for ViperX flow

    # For basket management (3 bottle slots + 1 random slot)
    bb.basket_poses = [
        {"command": "place_arm_pose"},  # Bottle slot 1
        {"command": "place_arm_pose"},  # Bottle slot 2
        {"command": "place_arm_pose"},  # Bottle slot 3
        {"command": "place_arm_pose"},  # Random items slot
    ]
    bb.basket_bottle_count = 0  # Track how many bottles have been picked
    
    # For shelf level state selection (3 hardcoded shelf commands)
    bb.shelf_height = None  # Semantic shelf level (1, 2, or 3)
    bb.shelf_poses = {
        1: {"command": "shelf_level_1_pose"},
        2: {"command": "shelf_level_2_pose"},
        3: {"command": "shelf_level_3_pose"},
    }
    bb.shelf_pose = None  # Selected preset shelf pose for the current level

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
