import py_trees
import robot_task_manager.bt_nodes as bt_nodes


def create_customer_viperx_tree(bb):
    """
    Customer picking task using ViperX arm with modular leaf nodes.
    
    Workflow:
    1. SetCurrentItem - get next item from order
    2. Navigate to shelf location
    3. Verify object position (detect via camera)
    4. Move to object and grab
    5. Move to basket slot (smart slot selection based on item type)
    6. Place in basket
    7. Return home
    8. Update inventory
    """

    search_timeout_sec = float(getattr(bb, "viperx_search_timeout_sec", 3.0))

    detect_center = py_trees.composites.Sequence(
        "DetectFromCenter",
        memory=True,
        children=[
            bt_nodes.RepositionViperXArm(goal_key="scan_center_pose", bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb, search_timeout_sec=search_timeout_sec),
        ],
    )

    detect_left = py_trees.composites.Sequence(
        "DetectFromLeft",
        memory=True,
        children=[
            bt_nodes.RepositionViperXArm(goal_key="scan_left_pose", bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb, search_timeout_sec=search_timeout_sec),
        ],
    )

    detect_right = py_trees.composites.Sequence(
        "DetectFromRight",
        memory=True,
        children=[
            bt_nodes.RepositionViperXArm(goal_key="scan_right_pose", bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb, search_timeout_sec=search_timeout_sec),
        ],
    )

    search_for_target = py_trees.composites.Selector(
        "SearchForTarget",
        memory=True,
        children=[
            detect_center,
            detect_left,
            detect_right,
        ],
    )

    detect_and_prepare = py_trees.composites.Sequence(
        "DetectAndPreparePick",
        memory=True,
        children=[
            search_for_target,
            bt_nodes.PrepareDetectedPickPoses(bb=bb),
        ],
    )

    # Old-style pick sequence: open -> pregrasp -> grasp(and close) -> lift
    approach_and_grab = py_trees.composites.Sequence(
        "ApproachAndGrab",
        memory=True,
        children=[
            bt_nodes.MoveViperXGripper(command="open", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="pregrasp_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="grasp_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="lift_pose", bb=bb),
        ],
    )

    # Sequence for placing item in basket
    place_in_basket = py_trees.composites.Sequence(
        "PlaceInBasket",
        memory=True,
        children=[
            bt_nodes.SelectBasketSlot(bb=bb),  # Select basket slot based on item type
            bt_nodes.RepositionViperXArm(goal_key="basket_pose", bb=bb),  # Move to basket position
            bt_nodes.MoveViperXGripper(command="open", bb=bb),  # Release item
            bt_nodes.RepositionViperXArm(goal_key="post_place_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="pre_return_pose", bb=bb),
        ],
    )

    # Main sequence for processing one item
    process_one_item = py_trees.composites.Sequence(
        "ProcessCustomerItem",
        memory=True,
        children=[
            # Set current item and update blackboard
            bt_nodes.SetCurrentItem(bb),
            bt_nodes.ResolveCurrentItemSemanticTargetViperX(bb),

            # Navigate to shelf location (x, y coordinates)
            bt_nodes.MaybeNavigateToGoalPose(goal_key="nav_goal", bb=bb),

            # Scan center -> left -> right until the target is seen, then build pick waypoints
            detect_and_prepare,

            # Execute pregrasp -> grasp(and close) -> lift
            approach_and_grab,

            # Place in basket with smart slot selection
            place_in_basket,

            # Return to home pose
            bt_nodes.RepositionViperXArm(goal_key="home_pose", bb=bb),

            # Update inventory
            bt_nodes.ChangeInventory(bb, mode="customer"),
        ],
    )

    num_items = len(getattr(bb, "items", []))

    # Repeat for each item in order
    repeat_each_item = py_trees.decorators.Repeat(
        name="RepeatForEachItem",
        child=process_one_item,
        num_success=num_items,
    )

    # Root sequence
    root = py_trees.composites.Sequence(
        "CustomerRoot",
        memory=True,
        children=[
            bt_nodes.DebugPrint("Customer ViperX tree selected"),
            bt_nodes.DebugPrint(lambda: f"Items in order: {len(getattr(bb, 'items', []))}"),
            repeat_each_item,
            
            # Go home after all items picked
            bt_nodes.SetHome(bb),
            bt_nodes.MaybeNavigateToGoalPose(goal_key="nav_goal", bb=bb),
            bt_nodes.DebugPrint("Customer picking complete"),
        ],
    )

    return root
