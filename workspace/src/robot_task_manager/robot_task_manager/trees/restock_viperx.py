import py_trees
import robot_task_manager.bt_nodes as bt_nodes

def create_restock_viperx_tree(bb):
    # Initial phase: pick first item and go to restock start pose
    root_sequence = py_trees.composites.Sequence("EmployeeRestockRoot", memory=True, children=[
        bt_nodes.SetCurrentItem(bb),
        bt_nodes.NavigateToGoalPose(goal_key="nav_goal", bb=bb),
        bt_nodes.MoveGripper(command="open"),
    ])

    # Per-item restock flow (one product type per submission; could be repeated qty times)
    item_sequence = py_trees.composites.Sequence("RestockItemSequence", memory=True, children=[
        py_trees.composites.Selector("RestockActionSelector", memory=True, children=[
            py_trees.composites.Sequence("PerformRestock", memory=True, children=[
                py_trees.composites.Sequence("PickAndPlace", memory=True, children=[
                    bt_nodes.RepositionArmToGoalPose(goal_key="basket_pose"),
                    bt_nodes.VerifyViperXPosition(),
                    bt_nodes.MoveGripper(command="close"),
                    bt_nodes.RepositionArmToGoalPose(goal_key="shelf_pose"),
                    bt_nodes.MoveGripper(command="open"),
                    bt_nodes.RepositionArmToGoalPose(goal_key="home_pose"),
                ]),
                bt_nodes.ChangeInventory(bb, mode="restock"),
            ]),
        ]),
    ])

    num_items = max(1, len(getattr(bb, "items", [])))
    repeat_items = py_trees.decorators.Repeat(
        name="RepeatUntilAllItemsDone",
        child=item_sequence,
        num_success=num_items,
    )

    cleanup_sequence = py_trees.composites.Sequence("RestockCleanup", memory=True, children=[
        bt_nodes.SetHome(bb),
        bt_nodes.NavigateToGoalPose(goal_key="nav_goal", bb=bb),
    ])

    root = py_trees.composites.Sequence("EmployeeRestockFlow", memory=True, children=[
        root_sequence,
        repeat_items,
        cleanup_sequence,
    ])

    return root
