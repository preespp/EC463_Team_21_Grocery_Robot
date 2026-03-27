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

    # Sequence for detecting object position and moving to it
    detect_and_approach = py_trees.composites.Sequence(
        "DetectAndApproach",
        memory=True,
        children=[
            bt_nodes.RepositionViperXArm(goal_key="pose"),  # Move to observation pose
            bt_nodes.VerifyViperXPosition(),  # Detect object, store in bb.detected_object_pose
        ],
    )

    # Sequence for placing item in basket
    place_in_basket = py_trees.composites.Sequence(
        "PlaceInBasket",
        memory=True,
        children=[
            bt_nodes.SelectBasketSlot(),  # Select basket slot based on item type
            bt_nodes.RepositionViperXArm(goal_key="basket_pose"),  # Move to basket position
            bt_nodes.MoveViperXGripper(command="open"),  # Release item
        ],
    )

    # Main sequence for processing one item
    process_one_item = py_trees.composites.Sequence(
        "ProcessCustomerItem",
        memory=True,
        children=[
            # Set current item and update blackboard
            bt_nodes.SetCurrentItem(bb),

            # Navigate to shelf location (x, y coordinates)
            bt_nodes.NavigateToGoalPose(goal_key="nav_goal"),

            # Detect object and verify positioning
            detect_and_approach,

            # Move to detected object pose and grab
            bt_nodes.MoveToDetectedPose(),
            bt_nodes.MoveViperXGripper(command="close"),

            # Place in basket with smart slot selection
            place_in_basket,

            # Return to home pose
            bt_nodes.RepositionViperXArm(goal_key="home_pose"),

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
            bt_nodes.NavigateToGoalPose(goal_key="nav_goal"),
            bt_nodes.DebugPrint("Customer picking complete"),
        ],
    )

    return root
