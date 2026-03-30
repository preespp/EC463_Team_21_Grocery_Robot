import py_trees
import robot_task_manager.bt_nodes as bt_nodes


def create_restock_viperx_tree(bb):
    process_one_item = py_trees.composites.Sequence(
        "ProcessRestockItem",
        memory=True,
        children=[
            bt_nodes.SetCurrentItem(bb),
            bt_nodes.ResolveCurrentItemSemanticTargetViperX(bb),
            bt_nodes.MaybeNavigateToGoalPose(goal_key="nav_goal", bb=bb),
            bt_nodes.MoveViperXGripper(command="open", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="basket_pose", bb=bb),
            bt_nodes.VerifyViperXPosition(bb=bb),
            bt_nodes.MoveViperXGripper(command="close", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="shelf_pose", bb=bb),
            bt_nodes.MoveViperXGripper(command="open", bb=bb),
            bt_nodes.RepositionViperXArm(goal_key="home_pose", bb=bb),
            bt_nodes.ChangeInventory(bb, mode="restock"),
        ],
    )

    num_items = max(1, len(getattr(bb, "items", [])))
    repeat_items = py_trees.decorators.Repeat(
        name="RepeatUntilAllItemsDone",
        child=process_one_item,
        num_success=num_items,
    )

    cleanup_sequence = py_trees.composites.Sequence(
        "RestockCleanup",
        memory=True,
        children=[
            bt_nodes.SetHome(bb),
            bt_nodes.MaybeNavigateToGoalPose(goal_key="nav_goal", bb=bb),
        ],
    )

    root = py_trees.composites.Sequence(
        "EmployeeRestockFlow",
        memory=True,
        children=[
            bt_nodes.DebugPrint("Restock ViperX tree selected"),
            repeat_items,
            cleanup_sequence,
        ],
    )

    return root
