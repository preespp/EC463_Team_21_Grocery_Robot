import py_trees
import robot_task_manager.bt_nodes as bt_nodes


class AlwaysFail(py_trees.behaviour.Behaviour):
    """Utility leaf for preserving overall task failure after recovery steps run."""

    def __init__(self, name: str = "AlwaysFail", message: str = "Preserving FAILURE after recovery"):
        super().__init__(name)
        self.message = message

    def update(self):
        self.feedback_message = self.message
        return py_trees.common.Status.FAILURE


class SetBlackboardBool(py_trees.behaviour.Behaviour):
    """Set a boolean blackboard flag and return SUCCESS."""

    def __init__(self, bb, key: str, value: bool, name: str | None = None):
        super().__init__(name or f"SetBlackboardBool[{key}]")
        self.bb = bb
        self.key = key
        self.value = bool(value)

    def update(self):
        setattr(self.bb, self.key, self.value)
        self.feedback_message = f"{self.key}={self.value}"
        return py_trees.common.Status.SUCCESS


class CheckBlackboardBool(py_trees.behaviour.Behaviour):
    """Return SUCCESS only when a boolean blackboard flag matches the expected value."""

    def __init__(self, bb, key: str, expected: bool, name: str | None = None):
        super().__init__(name or f"CheckBlackboardBool[{key}]")
        self.bb = bb
        self.key = key
        self.expected = bool(expected)

    def update(self):
        actual = bool(getattr(self.bb, self.key, False))
        self.feedback_message = f"{self.key}={actual}"
        if actual == self.expected:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


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
    grasp_retry_attempts = int(getattr(bb, "viperx_grasp_retry_attempts", 2))
    return_home_retry_attempts = int(getattr(bb, "viperx_return_home_retry_attempts", 2))
    order_items_completed_key = "customer_order_items_completed"

    setattr(bb, order_items_completed_key, False)

    detect_center = py_trees.composites.Sequence(
        "DetectFromCenter",
        memory=True,
        children=[
            bt_nodes.RepositionViperXArm(goal_key="scan_center_pose", bb=bb),
            bt_nodes.LockCurrentViperXOrientation(bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb, search_timeout_sec=search_timeout_sec),
            bt_nodes.PrepareWaistCenteringGoal(bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="waist_center_pose", bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb, search_timeout_sec=search_timeout_sec),
        ],
    )

    detect_left = py_trees.composites.Sequence(
        "DetectFromLeft",
        memory=True,
        children=[
            bt_nodes.RepositionViperXArm(goal_key="scan_left_pose", bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb, search_timeout_sec=search_timeout_sec),
            bt_nodes.PrepareWaistCenteringGoal(bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="waist_center_pose", bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb, search_timeout_sec=search_timeout_sec),
        ],
    )

    detect_right = py_trees.composites.Sequence(
        "DetectFromRight",
        memory=True,
        children=[
            bt_nodes.RepositionViperXArm(goal_key="scan_right_pose", bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb, search_timeout_sec=search_timeout_sec),
            bt_nodes.PrepareWaistCenteringGoal(bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="waist_center_pose", bb=bb),
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

    grab_motion = py_trees.composites.Sequence(
        "GrabMotion",
        memory=True,
        children=[
            bt_nodes.RepositionViperXArm(goal_key="pregrasp_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="grasp_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="post_grasp_lift_pose", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="post_lift_pose", bb=bb),
        ],
    )

    retry_grab_from_pregrasp = py_trees.decorators.Retry(
        name="RetryGrabFromPregrasp",
        child=grab_motion,
        num_failures=grasp_retry_attempts,
    )

    # Old-style pick sequence: open -> pregrasp -> grasp(and close) -> lift
    approach_and_grab = py_trees.composites.Sequence(
        "ApproachAndGrab",
        memory=True,
        children=[
            bt_nodes.MoveViperXGripper(command="open", bb=bb),
            retry_grab_from_pregrasp,
        ],
    )

    # Sequence for placing item in basket
    place_in_basket = py_trees.composites.Sequence(
        "PlaceInBasket",
        memory=True,
        children=[
            bt_nodes.SelectBasketSlot(bb=bb),  # Select the next free basket slot
            bt_nodes.RepositionViperXArm(goal_key="basket_pose", bb=bb),  # Move to basket position
            bt_nodes.MoveViperXGripper(command="open", bb=bb),  # Release item
            bt_nodes.MarkBasketSlotOccupied(bb=bb),
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

    mark_order_items_completed = SetBlackboardBool(
        bb=bb,
        key=order_items_completed_key,
        value=True,
        name="MarkCustomerOrderItemsCompleted",
    )

    success_return_home = py_trees.composites.Sequence(
        "ReturnHomeOnSuccess",
        memory=True,
        children=[
            bt_nodes.ClearLocalCostmapAndSmartBackUp(bb=bb),
            bt_nodes.SetHome(bb),
            py_trees.decorators.Retry(
                name="RetryReturnHomeOnSuccess",
                child=bt_nodes.MaybeNavigateToGoalPose(goal_key="nav_goal", bb=bb),
                num_failures=return_home_retry_attempts,
            ),
            bt_nodes.DebugPrint("Customer picking complete"),
        ],
    )

    main_flow = py_trees.composites.Sequence(
        "CustomerMainFlow",
        memory=True,
        children=[
            bt_nodes.DebugPrint("Customer ViperX tree selected"),
            bt_nodes.DebugPrint(lambda: f"Items in order: {len(getattr(bb, 'items', []))}"),
            repeat_each_item,
            mark_order_items_completed,
            success_return_home,
        ],
    )

    failure_return_home = py_trees.composites.Sequence(
        "ReturnHomeOnFailure",
        memory=True,
        children=[
            CheckBlackboardBool(
                bb=bb,
                key=order_items_completed_key,
                expected=False,
                name="GuardCustomerOrderFailedBeforeCompletion",
            ),
            bt_nodes.DebugPrint(
                lambda: (
                    "Customer pickup failed; attempting return home "
                    f"(retries={return_home_retry_attempts})"
                )
            ),
            bt_nodes.SetHome(bb),
            py_trees.decorators.Retry(
                name="RetryReturnHomeAfterFailure",
                child=bt_nodes.MaybeNavigateToGoalPose(goal_key="nav_goal", bb=bb),
                num_failures=return_home_retry_attempts,
            ),
            bt_nodes.DebugPrint(
                "Return-home recovery finished; preserving FAILED order result"
            ),
            AlwaysFail(),
        ],
    )

    root = py_trees.composites.Selector(
        "CustomerRoot",
        memory=True,
        children=[
            main_flow,
            failure_return_home,
        ],
    )

    return root
