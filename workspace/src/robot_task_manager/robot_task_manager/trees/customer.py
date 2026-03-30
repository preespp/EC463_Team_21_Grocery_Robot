import py_trees
import robot_task_manager.bt_nodes as bt_nodes


def create_customer_tree(bb):
    detect_shelf_target = bt_nodes.VerifyPosition(bb=bb, success_on_detect=True)

    align_until_ready = py_trees.decorators.Retry(
        name="AlignUntilReady",
        num_failures=5,
        child=py_trees.composites.Sequence(
            "AlignStep",
            memory=True,
            children=[
                bt_nodes.RepositionArmToGoalPose(goal_key="pose", bb=bb),
                bt_nodes.VerifyPosition(bb=bb),
            ],
        ),
    )

    process_one_item = py_trees.composites.Sequence(
        "ProcessCustomerItem",
        memory=True,
        children=[
            bt_nodes.SetCurrentItem(bb),
            bt_nodes.ResolveCurrentItemSemanticTarget(bb),
            bt_nodes.NavigateToGoalPose(goal_key="nav_goal", bb=bb),
            bt_nodes.RepositionRackToGoalLevel(goal_key="rack_goal"),
            detect_shelf_target,
            align_until_ready,
            bt_nodes.MoveGripper(command="close"),
            bt_nodes.RepositionArmToGoalPose(goal_key="basket_pose", bb=bb),
            bt_nodes.RepositionRackToGoalLevel(goal_key="home_rack"),
            bt_nodes.MoveGripper(command="open"),
            bt_nodes.RepositionArmToGoalPose(goal_key="home_pose", bb=bb),
            bt_nodes.ChangeInventory(bb, mode="customer"),
        ],
    )

    num_items = max(1, len(getattr(bb, "items", [])))

    repeat_each_item = py_trees.decorators.Repeat(
        name="RepeatUntilAllItemsDone",
        child=process_one_item,
        num_success=num_items,
    )

    root = py_trees.composites.Sequence(
        "CustomerRoot",
        memory=True,
        children=[
            bt_nodes.DebugPrint("Customer tree selected"),
            bt_nodes.DebugPrint(lambda: f"Items in order: {len(getattr(bb, 'items', []))}"),
            repeat_each_item,
            bt_nodes.SetHome(bb),
            bt_nodes.NavigateToGoalPose(goal_key="nav_goal", bb=bb),
            bt_nodes.DebugPrint("Customer picking complete"),
        ],
    )

    return root
