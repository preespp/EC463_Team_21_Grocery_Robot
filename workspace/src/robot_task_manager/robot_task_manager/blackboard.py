from copy import deepcopy

import py_trees
from robot_interfaces.msg import Order


DEFAULT_VIPERX_BASE_FRAME = "vx300s/base_link"


def _make_pose(x: float, y: float, z: float, frame_id: str = "base_link") -> dict:
    return {
        "frame_id": frame_id,
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
    }


def _make_command_target(command: str) -> dict:
    return {"command": str(command)}


def reset_viperx_manipulation_state(bb):
    bb.pose = deepcopy(
        getattr(bb, "observation_pose", _make_command_target("startup_arm_pose"))
    )
    bb.basket_pose = deepcopy(
        getattr(bb, "default_basket_pose", _make_command_target("place_arm_pose_1"))
    )
    bb.goal_pose = None
    bb.shelf_pose = None
    bb.shelf_height = None
    bb.detected_object_pose = None
    bb.pregrasp_pose = None
    bb.grasp_pose = None
    bb.post_grasp_lift_pose = None
    bb.lift_pose = deepcopy(
        getattr(bb, "default_lift_pose", _make_command_target("lift_arm_pose"))
    )
    bb.post_lift_pose = deepcopy(
        getattr(bb, "default_post_lift_pose", _make_command_target("post_lift_arm_pose"))
    )
    bb.locked_pick_orientation_xyzw = None
    bb.waist_center_pose = _make_command_target("waist_delta_arm_pose")
    if not hasattr(bb, "basket_next_slot_index"):
        bb.basket_next_slot_index = 0
    bb.current_basket_slot_index = None


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
    bb.arm_base_frame = DEFAULT_VIPERX_BASE_FRAME
    bb.viperx_ee_orientation_frame = "vx300s/ee_gripper_link"
    bb.viperx_ee_link = "vx300s/ee_gripper_link"
    # Lock the real EE quaternion from the known-horizontal center scan pose.
    # This is more reliable on hardware than assuming identity is horizontal.
    bb.viperx_use_current_ee_orientation = True
    bb.viperx_fixed_pick_orientation_xyzw = None
    bb.viperx_target_classes_text = "roasted tea,green tea,water,can,apple,orange,lemon,bag of chips"
    bb.viperx_close_gripper_position = 0.040
    bb.viperx_close_gripper_positions = {
        "bag of chips": 0.034,
        "chips": 0.034,
        "chip": 0.034,
        "green tea": 0.040,
        "roasted tea": 0.040,
        "water": 0.038,
        "can": 0.045,
        "apple": 0.051,
        "orange": 0.052,
        "lemon": 0.046,
    }
    bb.viperx_grasp_offset_z_by_item_m = {
        "can": 0.020,
        "apple": 0.030,
        "orange": 0.030,
        "lemon": 0.020,
    }
    bb.viperx_pregrasp_target_x_m = 0.35
    bb.viperx_pregrasp_offset_x_m = -0.15
    bb.viperx_pregrasp_offset_z_m = 0.0
    bb.viperx_grasp_offset_x_m = 0.02
    bb.viperx_grasp_offset_z_m = 0.00
    bb.viperx_post_grasp_lift_offset_z_m = 0.10
    bb.viperx_lift_offset_z_m = 0.20
    bb.viperx_search_timeout_sec = 3.0
    bb.viperx_workspace_min_x = -0.50
    bb.viperx_workspace_max_x = 0.80
    bb.viperx_workspace_min_y = -0.80
    bb.viperx_workspace_max_y = 0.80
    bb.viperx_workspace_min_z = -0.20
    bb.viperx_workspace_max_z = 0.80
    bb.viperx_target_stability_count = 4
    bb.viperx_target_match_distance_m = 0.03
    bb.viperx_cooldown_sec = 3.0
    bb.viperx_grasp_retry_attempts = 5
    bb.viperx_waist_centering_enabled = True
    bb.viperx_waist_centering_gain = 1.0
    bb.viperx_waist_centering_sign = -1.0
    bb.viperx_waist_centering_min_error_rad = 0.04
    bb.viperx_waist_centering_max_delta_rad = 0.35
    bb.scan_center_pose = _make_command_target("scan_center_arm_pose")
    bb.scan_left_pose = _make_command_target("scan_left_arm_pose")
    bb.scan_right_pose = _make_command_target("scan_right_arm_pose")
    bb.observation_pose = deepcopy(bb.scan_center_pose)
    bb.home_pose = _make_command_target("return_arm_pose")
    bb.restock_pick_ready_pose = _make_command_target("restock_pick_ready_arm_pose")
    bb.restock_pick_approach_pose = _make_command_target("restock_pick_approach_arm_pose")
    bb.restock_pick_pose = _make_command_target("restock_pick_arm_pose")
    bb.restock_post_pick_pose = _make_command_target("restock_post_pick_arm_pose")
    bb.restock_transfer_pose = _make_command_target("restock_transfer_arm_pose")
    bb.restock_place_pose = _make_command_target("restock_place_arm_pose")
    bb.restock_home_pose = _make_command_target("restock_home_arm_pose")
    bb.default_lift_pose = _make_command_target("lift_arm_pose")
    bb.default_post_lift_pose = _make_command_target("post_lift_arm_pose")
    bb.default_basket_pose = _make_command_target("place_arm_pose_1")
    bb.post_place_pose = _make_command_target("post_place_arm_pose")
    bb.pre_return_pose = _make_command_target("pre_return_arm_pose")
    bb.post_lift_pose = deepcopy(bb.default_post_lift_pose)
    bb.goal_pose = None  # Preferred BT arm target pose key for ViperX flow

    # For basket management (6 physical basket positions used sequentially)
    bb.basket_poses = [
        _make_command_target("place_arm_pose_1"),
        _make_command_target("place_arm_pose_2"),
        _make_command_target("place_arm_pose_3"),
        _make_command_target("place_arm_pose_4"),
        _make_command_target("place_arm_pose_5"),
        _make_command_target("place_arm_pose_6"),
    ]
    bb.basket_next_slot_index = 0
    bb.current_basket_slot_index = None
    
    # For shelf level state selection (3 hardcoded shelf commands)
    bb.shelf_height = None  # Semantic shelf level (1, 2, or 3)
    bb.shelf_poses = {
        1: _make_command_target("shelf_level_1_pose"),
        2: _make_command_target("shelf_level_2_pose"),
        3: _make_command_target("shelf_level_3_pose"),
    }
    bb.shelf_pose = None  # Selected preset shelf pose for the current level

    # For object detection results
    bb.detected_object_pose = None  # Pose returned from vision service
    bb.latest_detections_json = None
    bb.latest_detection_time = None
    bb.shared_tf_buffer = None

    # For rack levels
    bb.current_rack = 1
    bb.rack_goal = 1
    bb.home_rack = 1

    # For navigation
    bb.skip_navigation = False
    bb.nav_goal = (0.0, 0.0, 0.0)
    bb.home_goal = (0.0, 0.0, 0.0)
    bb.slot_id = None
    bb.anchor_id = None
    bb.rack_id = None
    bb.semantic_id = None
    bb.semantic_target_label = None
    bb.nav_goal_source = None
    bb.viperx_detection_min_confidence = 0.40

    reset_viperx_manipulation_state(bb)

    return bb


def setup_custom_blackboard():
    bb = py_trees.blackboard.Blackboard()

    bb.mode = None

    bb.order: Order | None = None
    bb.order_id_text = None

    bb.items = []
    bb.item_index = 0
    bb.current_item = None
    bb.num_current_item = 0

    # Servo-arm blackboard uses Cartesian targets in base_link.
    # robot_manipulation converts these into joint targets for the motors.
    bb.home_pose = _make_pose(0.18, 0.00, 0.28)
    bb.basket_pose = _make_pose(0.16, -0.18, 0.16)
    bb.pose = _make_pose(0.30, 0.00, 0.18)
    bb.goal_pose = None
    bb.shelf_pose = None
    bb.arm_last_commanded_pose = None

    bb.detected_object_pose = None

    bb.current_rack = 1
    bb.rack_goal = 1
    bb.home_rack = 1

    bb.skip_navigation = False
    bb.nav_goal = (0.0, 0.0, 0.0)
    bb.home_goal = (0.0, 0.0, 0.0)
    bb.slot_id = None
    bb.anchor_id = None
    bb.rack_id = None
    bb.semantic_id = None
    bb.semantic_target_label = None
    bb.nav_goal_source = None
    bb.viperx_detection_min_confidence = 0.40

    return bb
